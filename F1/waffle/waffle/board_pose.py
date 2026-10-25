import math

import cv2
import numpy as np

class BoardPose:
    """ID 배치: 위쪽 0, 1 / 아래쪽 2, 3."""

    def __init__(self, marker_m, gap_m, matrix, distortion):
        if not math.isfinite(marker_m) or marker_m <= 0:
            raise ValueError("마커 길이는 양수여야 합니다.")
        if not math.isfinite(gap_m) or gap_m < 0:
            raise ValueError("마커 간격은 0 이상이어야 합니다.")

        self.matrix = matrix
        self.distortion = distortion
        self.ids = (0, 1, 2, 3)

        # 보드 중심을 원점으로 설정
        # x: 오른쪽, y: 아래쪽, z: 보드 평면 밖
        half_marker = marker_m / 2.0
        center_offset = (marker_m + gap_m) / 2.0

        centers = {
            0: (-center_offset, -center_offset),
            1: (+center_offset, -center_offset),
            2: (-center_offset, +center_offset),
            3: (+center_offset, +center_offset),
        }

        # 검출된 ArUco 모서리 순서에 맞춤:
        # 좌상 → 우상 → 우하 → 좌하
        local_corners = np.array(
            [
                [-half_marker, -half_marker, 0.0],
                [+half_marker, -half_marker, 0.0],
                [+half_marker, +half_marker, 0.0],
                [-half_marker, +half_marker, 0.0],
            ],
            dtype=np.float32,
        )

        self.object_corners = {}
        for marker_id, (x, y) in centers.items():
            self.object_corners[marker_id] = (
                local_corners
                + np.array([x, y, 0.0], dtype=np.float32)
            )

    def estimate(self, corners, ids):
        """성공 시 계산 결과, 실패 시 None과 이유를 반환."""
        if ids is None:
            return None, "no_markers"

        detected_ids = np.asarray(ids).reshape(-1)
        if len(corners) != len(detected_ids):
            return None, "corner_count_mismatch"

        object_points = []
        image_points = []

        for marker_id in self.ids:
            indices = np.flatnonzero(detected_ids == marker_id)

            if len(indices) == 0:
                return None, f"missing_id_{marker_id}"
            if len(indices) > 1:
                return None, f"duplicate_id_{marker_id}"

            points = np.asarray(
                corners[int(indices[0])], dtype=np.float32
            ).reshape(4, 2)

            if not np.isfinite(points).all():
                return None, "invalid_corner"

            object_points.append(self.object_corners[marker_id])
            image_points.append(points)

        # 마커 4개 × 모서리 4개 = 총 16개 대응점
        object_points = np.concatenate(object_points)
        image_points = np.concatenate(image_points)

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok:
            return None, "pose_failed"

        if not (
            np.isfinite(rvec).all()
            and np.isfinite(tvec).all()
        ):
            return None, "invalid_pose"

        rotation, _ = cv2.Rodrigues(rvec)
        translation = tvec.reshape(3)

        # 모든 보드 모서리가 카메라 앞에 있는지 확인
        camera_points = (
            object_points @ rotation.T + translation
        )
        if np.any(camera_points[:, 2] <= 0):
            return None, "board_behind_camera"

        # 추정 자세로 모서리를 다시 영상에 투영
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.matrix,
            self.distortion,
        )

        errors = (
            image_points - projected.reshape(-1, 2)
        )
        squared_errors = np.sum(errors ** 2, axis=1)
        rms_px = float(np.sqrt(np.mean(squared_errors)))

        # 한 마커의 배치 오류가 전체 평균에 가려지지 않도록 검사
        per_marker_rms = np.sqrt(
            squared_errors.reshape(4, 4).mean(axis=1)
        )

        # 초기 시험 기준이며 실제 정확도를 보장하는 값은 아님
        if np.max(per_marker_rms) > 1.5:
            return None, "reprojection_error"

        # 보드 평면의 법선 방향
        normal = rotation[:, 2].copy()

        # 법선이 보드에서 카메라 쪽을 향하도록 부호 통일
        if np.dot(normal, translation) > 0:
            normal = -normal

        # 카메라 원점과 보드 평면 사이의 수직 거리
        plane_distance = -float(np.dot(normal, translation))

        # 보드에 수직으로 접근하는 방향의 수평 각도
        # 카메라 좌표 기준: 오른쪽 방향을 양수로 표시
        approach = -normal
        heading_error = math.atan2(
            float(approach[0]),
            float(approach[2]),
        )

        # 가상 보드 중심인 (0, 0, 0)을 영상에 투영
        center_image, _ = cv2.projectPoints(
            np.zeros((1, 3), dtype=np.float32),
            rvec,
            tvec,
            self.matrix,
            self.distortion,
        )

        result = {
            "center_uv": center_image.reshape(2),
            "x_m": float(translation[0]),
            "y_m": float(translation[1]),
            "z_m": float(translation[2]),
            "plane_distance_m": plane_distance,
            "heading_error_deg": math.degrees(heading_error),
            "reprojection_rms_px": rms_px,
        }

        return result, "ok"