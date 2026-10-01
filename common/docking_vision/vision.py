"""Algorithms adapted from docs/lecture_original/pose_ArUco (PinkLAB).

Images stay at their native size. Coordinates are camera optical coordinates:
+x right, +y down, +z forward. No robot velocity commands are produced.
"""
from pathlib import Path
import math

import cv2
import numpy as np
import yaml


def positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def dictionary(name):
    if not name.startswith('DICT_') or not hasattr(cv2.aruco, name):
        raise ValueError(f'Unsupported ArUco dictionary: {name}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_marker(name, marker_id, pixels=600, margin=60):
    d = dictionary(name)
    if not 0 <= marker_id < len(d.bytesList) or pixels < 32 or margin < 1:
        raise ValueError('Invalid marker ID, pixels (<32), or margin (<1)')
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(d, marker_id, pixels)
    else:
        img = cv2.aruco.drawMarker(d, marker_id, pixels)
    return cv2.copyMakeBorder(img, margin, margin, margin, margin,
                              cv2.BORDER_CONSTANT, value=255)


def board_corners(image, cols, rows):
    if cols < 2 or rows < 2:
        raise ValueError('Use INTERNAL corner counts >= 2')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows))
    if not ok:
        return None
    # winSize is a HALF window: (11, 11) searches 23x23 pixels.
    # Small/foreshortened squares can put adjacent corners inside that window.
    # Size it from the tightest observed grid spacing, with an upper bound.
    grid = corners.reshape(rows, cols, 2)
    spacing = np.concatenate((
        np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
        np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel(),
    ))
    radius = max(1, min(5, int(float(spacing.min()) * 0.3)))
    return cv2.cornerSubPix(gray, corners, (radius, radius), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .001))


def calibrate(paths, cols, rows, square_m, camera_name, min_views=10):
    positive(square_m, 'square_m')
    if cols < 2 or rows < 2 or min_views < 3:
        raise ValueError('Corner counts must be >=2, min_views >=3')
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    objects, points, accepted, rejected = [], [], [], []
    size = None
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            rejected.append(str(path))
            continue
        current = (image.shape[1], image.shape[0])
        if size is not None and current != size:
            raise ValueError(f'Mixed calibration resolutions: {size} vs {current}: {path}')
        size = current
        corners = board_corners(image, cols, rows)
        if corners is None:
            rejected.append(str(path))
            continue
        objects.append(obj.copy())
        points.append(corners)
        accepted.append(str(path))
    if len(points) < min_views:
        raise ValueError(f'Only {len(points)} usable views; need {min_views}; rejected: {len(rejected)}')
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        objects, points, size, None, None)
    errors = []
    for objp, imgp, rvec, tvec in zip(objects, points, rotations, translations):
        projected, _ = cv2.projectPoints(objp, rvec, tvec, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((imgp - projected) ** 2, axis=2)))))
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or not math.isfinite(rms):
        raise ValueError('Calibration produced non-finite results')
    return {
        'image_width': size[0], 'image_height': size[1], 'camera_name': camera_name,
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': matrix.ravel().tolist()},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': distortion.size,
                                    'data': distortion.ravel().tolist()},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).ravel().tolist()},
        'projection_matrix': {'rows': 3, 'cols': 4,
                              'data': np.column_stack((matrix, np.zeros(3))).ravel().tolist()},
        'rms_px': float(rms), 'per_view_rms_px': errors,
        'corner_refinement': 'spacing_limited_subpix_v1',
        'board': {'cols': cols, 'rows': rows, 'square_m': square_m},
        'accepted_images': accepted, 'rejected_images': rejected,
    }


def load_calibration(path):
    data = yaml.safe_load(Path(path).read_text())
    matrix = np.array(data['camera_matrix']['data'], dtype=float).reshape(3, 3)
    distortion = np.array(data['distortion_coefficients']['data'], dtype=float).ravel()
    size = (int(data['image_width']), int(data['image_height']))
    if (data.get('distortion_model') != 'plumb_bob' or distortion.size != 5 or
            not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or
            matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or min(size) <= 0 or
            not np.allclose(matrix[2], [0, 0, 1])):
        raise ValueError('Invalid calibration or unsupported distortion model (use plumb_bob, 5 coefficients)')
    return matrix, distortion, size


class MarkerFinder:
    """One image pass shared by single-marker and four-marker detection."""

    def __init__(self, name):
        self.dictionary = dictionary(name)
        self.params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
                       else cv2.aruco.DetectorParameters_create())
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.params)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)

    def find(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is None:
            return cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        return self.detector.detectMarkers(gray)


class Detector:
    def __init__(self, name='DICT_5X5_100', target_id=23, marker_m=None, calibration=None):
        self.finder = MarkerFinder(name)
        self.dictionary = self.finder.dictionary
        if not 0 <= target_id < len(self.dictionary.bytesList):
            raise ValueError('Target ID outside dictionary')
        if (marker_m is None) != (calibration is None):
            raise ValueError('Pose estimation requires BOTH calibration and actual marker length')
        self.target_id = target_id
        self.marker_m = positive(marker_m, 'marker_m') if marker_m is not None else None
        self.calibration = calibration

    def detect(self, frame):
        h, w = frame.shape[:2]
        if self.calibration is not None and self.calibration[2] != (w, h):
            raise ValueError(f'Calibration resolution {self.calibration[2]} != image {(w, h)}. Recalibrate; do not resize.')
        corners, ids, _ = self.finder.find(frame)
        result = {'target_id': self.target_id, 'detected': False, 'pose_valid': False,
                  'image_width': w, 'image_height': h, 'visible_ids': [],
                  'center_px': None, 'horizontal_error_px': None,
                  'bearing_rad': None, 'tvec_m': None, 'rvec_rad': None,
                  'normal_yaw_rad': None, 'reprojection_rms_px': None}
        output = frame.copy()
        if ids is None:
            return result, output
        result['visible_ids'] = ids.flatten().tolist()
        cv2.aruco.drawDetectedMarkers(output, corners, ids)
        indices = np.flatnonzero(ids.flatten() == self.target_id)
        # Two identical IDs give ambiguous docking targets: reject both.
        if len(indices) != 1:
            return result, output
        points = corners[int(indices[0])].reshape(4, 2)
        center = points.mean(axis=0)
        cx = self.calibration[0][0, 2] if self.calibration is not None else w / 2
        error = float(center[0] - cx)
        result.update(detected=True, center_px=center.tolist(), horizontal_error_px=error)
        cv2.line(output, (int(cx), 0), (int(cx), h - 1), (0, 255, 255), 1)
        cv2.circle(output, tuple(np.rint(center).astype(int)), 5, (0, 0, 255), -1)
        if self.calibration is not None:
            matrix, distortion, _ = self.calibration
            s = self.marker_m / 2
            obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
            ok, rvec, tvec = cv2.solvePnP(obj, points, matrix, distortion,
                                         flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok and np.isfinite(rvec).all() and np.isfinite(tvec).all() and tvec[2, 0] > 0:
                projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)
                reproj = float(np.sqrt(np.mean(np.sum((points - projected.reshape(4, 2)) ** 2, axis=1))))
                rotation, _ = cv2.Rodrigues(rvec)
                # Zero for a frontal marker (its +z normal points back to camera).
                normal = rotation[:, 2]
                result.update(pose_valid=True, tvec_m=tvec.ravel().tolist(),
                              rvec_rad=rvec.ravel().tolist(),
                              bearing_rad=math.atan2(float(tvec[0, 0]), float(tvec[2, 0])),
                              normal_yaw_rad=math.atan2(-float(normal[0]), -float(normal[2])),
                              reprojection_rms_px=reproj)
                cv2.drawFrameAxes(output, matrix, distortion, rvec, tvec, self.marker_m * .5)
        cv2.putText(output, f'ID {self.target_id} dx={error:+.1f}px', (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
        return result, output
