# ========================================================================
# 역할: 4마커 도킹 보드의 형상 정의와 자세 추정 (카메라 기준 보드 위치·각도).
# 사용처: docking_vision_worker.py(BoardDetector), docking_config.py, generate_station_boards.py, cli.py.
# ========================================================================
"""Joint pose of a measured planar four-marker docking board (no motion commands)."""
from pathlib import Path
import math

import cv2
import numpy as np
import yaml

from .vision import MarkerFinder, dictionary, positive


# 보드 정의: 원점=도킹 중심, x 오른쪽, y 위, z 보드 밖. 마커 ID·크기·위치.
class DockingBoard:
    """Origin is the desired docking center; x right, y up, z out of board."""

    # 보드 사양(마커 목록) 검사·저장.
    def __init__(self, data):
        if not isinstance(data, dict) or not isinstance(data.get('dictionary'), str):
            raise ValueError('Board dictionary must be specified')
        self.dictionary_name = data['dictionary']
        size = len(dictionary(self.dictionary_name).bytesList)
        markers = data.get('markers')
        if not isinstance(markers, list) or len(markers) != 4:
            raise ValueError('Exactly four measured markers are required')
        self.points = {}
        self.lengths = {}
        for marker in markers:
            if not isinstance(marker, dict):
                raise ValueError('Each marker must be a mapping')
            marker_id = marker.get('id')
            if type(marker_id) is not int or not 0 <= marker_id < size:
                raise ValueError('Each marker needs a valid integer ID')
            if marker_id in self.points:
                raise ValueError('The four marker IDs must be distinct')
            try:
                length = positive(float(marker['length_m']), 'marker length_m')
                center = np.array(marker['center_xy_m'], dtype=float)
                angle = math.radians(float(marker.get('rotation_deg', 0)))
            except (KeyError, TypeError) as exc:
                raise ValueError('Set measured length_m and center_xy_m for every marker') from exc
            if center.shape != (2,) or not np.isfinite(center).all() or not math.isfinite(angle):
                raise ValueError('Invalid marker center or rotation')
            s = length / 2
            xy = np.array([[-s, s], [s, s], [s, -s], [-s, -s]], dtype=float)
            rotation = np.array([[math.cos(angle), -math.sin(angle)],
                                 [math.sin(angle), math.cos(angle)]])
            xy = xy @ rotation.T + center
            self.points[marker_id] = np.column_stack((xy, np.zeros(4))).astype(np.float32)
            self.lengths[marker_id] = length
        self.ids = list(self.points)
        self.spec = {'dictionary': self.dictionary_name, 'markers': markers}
        for i, a in enumerate(self.ids):
            for b in self.ids[i+1:]:
                area, _ = cv2.intersectConvexConvex(self.points[a][:, :2].copy(),
                                                   self.points[b][:, :2].copy())
                if area > 1e-9:
                    raise ValueError('Marker areas overlap; check board measurements')
        # Configurable commissioning values, not claims of robot accuracy.
        self.max_error = positive(float(data.get('max_reprojection_rms_px', 1.5)), 'max reprojection error')
        self.lateral_tolerance = positive(float(data.get('lateral_tolerance_m', .005)), 'lateral tolerance')
        self.yaw_tolerance = math.radians(positive(float(data.get('yaw_tolerance_deg', 2)), 'yaw tolerance'))

    # 보드 YAML 파일 읽기.
    @classmethod
    def load(cls, path):
        return cls(yaml.safe_load(Path(path).read_text()))


# 자세(rvec/tvec)를 관측 사전 항목으로 변환.
def pose_fields(rvec, tvec):
    rotation, _ = cv2.Rodrigues(rvec)
    normal = rotation[:, 2]
    return {'tvec_m': tvec.ravel().tolist(), 'rvec_rad': rvec.ravel().tolist(),
            'bearing_rad': math.atan2(float(tvec[0, 0]), float(tvec[2, 0])),
            'normal_yaw_rad': math.atan2(-float(normal[0]), -float(normal[2]))}


# 마커 코너들로 보드 자세 계산 (solvePnP).
def fit_pose(objects, images, calibration):
    matrix, distortion, _ = calibration
    ok, rvec, tvec = cv2.solvePnP(objects, images, matrix, distortion,
                                 flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    camera_points = objects @ rotation.T + tvec.reshape(1, 3)
    if np.any(camera_points[:, 2] <= 0):
        return None
    projected, _ = cv2.projectPoints(objects, rvec, tvec, matrix, distortion)
    residual = np.linalg.norm(images.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)
    return rvec, tvec, residual


# 한 이미지에서 보드 마커를 찾고 자세를 계산하는 검출기.
class BoardDetector:
    # 보드·카메라 보정값 설정.
    def __init__(self, board, calibration):
        if calibration is None:
            raise ValueError('Four-marker joint pose requires camera calibration')
        self.board = board
        self.calibration = calibration
        self.finder = MarkerFinder(board.dictionary_name)

    # 이미지 1장 처리 → (관측 사전, 주석 그린 이미지).
    def detect(self, frame):
        h, w = frame.shape[:2]
        if self.calibration[2] != (w, h):
            raise ValueError(f'Calibration resolution {self.calibration[2]} != image {(w, h)}')
        corners, ids, _ = self.finder.find(frame)
        output = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(output, corners, ids)
        result = self.estimate(corners, [] if ids is None else ids.ravel().tolist())
        if result['pose_valid']:
            cv2.drawFrameAxes(output, *self.calibration[:2],
                             np.array(result['rvec_rad']), np.array(result['tvec_m']),
                             min(self.board.lengths.values()))
        label = f"Board {len(result['detected_ids'])}/4: {result['reason']}"
        cv2.putText(output, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 200, 255), 2)
        # Fixed image-center reference, drawn only on the display copy.
        # This is the frame center, not the calibrated optical principal point.
        center = (w // 2, h // 2)
        cv2.drawMarker(output, center, (0, 0, 0), cv2.MARKER_CROSS, 28, 2, cv2.LINE_AA)
        cv2.drawMarker(output, center, (0, 255, 255), cv2.MARKER_CROSS, 28, 1, cv2.LINE_AA)
        return result, output

    # 찾은 마커들로 보드 자세 추정 (마커 순서와 무관).
    def estimate(self, corners, ids):
        """Accept canonical corner order, regardless of marker detection ordering."""
        if len(corners) != len(ids):
            raise ValueError('Corner/ID count mismatch')
        ids = [int(i) for i in ids]
        result = {'mode': 'four_marker_board', 'target_ids': self.board.ids,
                  'board_spec': self.board.spec,
                  'visible_ids': ids, 'detected_ids': [], 'missing_ids': [],
                  'duplicate_ids': [], 'markers': [], 'detected': False,
                  'pose_valid': False, 'alignment_ready': False, 'reason': 'markers_missing',
                  'image_width': self.calibration[2][0], 'image_height': self.calibration[2][1],
                  'tvec_m': None, 'rvec_rad': None, 'bearing_rad': None,
                  'normal_yaw_rad': None, 'horizontal_error_m': None,
                  'horizontal_error_px': None, 'reprojection_rms_px': None}
        found = {}
        for marker_id in self.board.ids:
            indices = [i for i, value in enumerate(ids) if value == marker_id]
            item = {'id': marker_id, 'detected': len(indices) == 1,
                    'center_px': None, 'pose_valid': False, 'tvec_m': None,
                    'rvec_rad': None, 'reprojection_rms_px': None}
            if not indices:
                result['missing_ids'].append(marker_id)
            elif len(indices) != 1:
                result['duplicate_ids'].append(marker_id)
            else:
                pts = np.asarray(corners[indices[0]], np.float32).reshape(4, 2)
                if not np.isfinite(pts).all():
                    raise ValueError('Non-finite marker corners')
                found[marker_id] = pts
                result['detected_ids'].append(marker_id)
                item['center_px'] = pts.mean(axis=0).tolist()
                local_obj = self.board.points[marker_id] - self.board.points[marker_id].mean(axis=0)
                fit = fit_pose(local_obj, pts, self.calibration)
                if fit is not None:
                    rvec, tvec, errors = fit
                    error = float(np.sqrt(np.mean(errors**2)))
                    item.update(pose_fields(rvec, tvec), pose_valid=error <= self.board.max_error,
                                reprojection_rms_px=error)
            result['markers'].append(item)
        if result['duplicate_ids']:
            result['reason'] = 'duplicate_target_id'
            return result
        # Partial views still report each marker; never silently become a one-marker board.
        if len(found) != 4:
            return result
        result['detected'] = True
        objects = np.concatenate([self.board.points[i] for i in self.board.ids])
        images = np.concatenate([found[i] for i in self.board.ids])
        fit = fit_pose(objects, images, self.calibration)
        if fit is None:
            result['reason'] = 'pose_failed'
            return result
        rvec, tvec, errors = fit
        error = float(np.sqrt(np.mean(errors**2)))
        per_marker = np.sqrt(np.mean(errors.reshape(4, 4)**2, axis=1))
        result['reprojection_rms_px'] = error
        for marker, value in zip(result['markers'], per_marker):
            marker['board_reprojection_rms_px'] = float(value)
        if max(per_marker) > self.board.max_error:
            result['reason'] = 'board_geometry_inconsistent'
            return result
        fields = pose_fields(rvec, tvec)
        result.update(fields, pose_valid=True, horizontal_error_m=float(tvec[0, 0]))
        center, _ = cv2.projectPoints(np.zeros((1, 3)), rvec, tvec, *self.calibration[:2])
        result['horizontal_error_px'] = float(center[0, 0, 0] - self.calibration[0][0, 2])
        result['alignment_ready'] = bool(abs(tvec[0, 0]) <= self.board.lateral_tolerance and
                                        abs(fields['normal_yaw_rad']) <= self.board.yaw_tolerance)
        result['reason'] = 'aligned_observation' if result['alignment_ready'] else 'alignment_needed'
        return result
