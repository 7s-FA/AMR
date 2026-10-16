# ========================================================================
# 역할: 카메라 점검·보정 도구 (카메라 확인 → 체커보드 촬영 → 보정 계산 → ArUco 관측 확인). 운용 중에는 쓰지 않는다.
# 실행: 수동. PYTHONPATH 에 카메라 폴더를 넣고 python3 -m docking_vision.cli <명령>.
# ========================================================================
"""Camera check -> checkerboard capture -> calibration -> ArUco observations."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

from .source import RosSource, VideoSource
from .board import BoardDetector, DockingBoard
from .vision import Detector, board_corners, calibrate, load_calibration, make_marker, positive


# 이미지 파일 저장.
def write_image(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f'Failed to save {path}')


# 명령행 인자 정의.
def parser():
    p = argparse.ArgumentParser(description=__doc__)
    commands = p.add_subparsers(dest='command', required=True)
    for name in ('probe', 'capture', 'detect'):
        cmd = commands.add_parser(name)
        cmd.add_argument('--source', help='Local USB index (/dev/video0: use 0) or video file; default: ROS')
        cmd.add_argument('--topic', default='/burger2/camera/image_raw')
        cmd.add_argument('--compressed', action='store_true', help='Topic is sensor_msgs/CompressedImage')
        cmd.add_argument('--timeout', type=float, default=5, help='Maximum seconds without a new frame')
        cmd.add_argument('--show', action='store_true', help='Show window, q quits; capture: s saves')
        if name == 'probe':
            cmd.add_argument('--frames', type=int, default=60)
            cmd.add_argument('--output', default='data/burger2/camera_test.jpg')
        elif name == 'capture':
            cmd.add_argument('--output', default='data/burger2/calibration_images')
            cmd.add_argument('--cols', type=int, required=True)
            cmd.add_argument('--rows', type=int, required=True)
            cmd.add_argument('--count', type=int, default=20)
            cmd.add_argument('--interval', type=float, default=2, help='Automatic save interval without --show')
        else:
            cmd.add_argument('--dictionary', help='Single-marker dictionary (default DICT_5X5_100)')
            cmd.add_argument('--target-id', type=int, help='Single-marker ID (default 23)')
            cmd.add_argument('--board', help='Measured four-marker board YAML; requires --calibration')
            cmd.add_argument('--marker-m', type=float)
            cmd.add_argument('--calibration')
            cmd.add_argument('--frames', type=int, default=0, help='0 runs until Ctrl+C/q')
            cmd.add_argument('--observation-topic', help='Publish JSON std_msgs/String when using ROS input')
    cmd = commands.add_parser('calibrate')
    cmd.add_argument('--images', required=True)
    cmd.add_argument('--cols', type=int, required=True)
    cmd.add_argument('--rows', type=int, required=True)
    cmd.add_argument('--square-m', type=float, required=True)
    cmd.add_argument('--camera-name', default='burger2_camera')
    cmd.add_argument('--min-views', type=int, default=10)
    cmd.add_argument('--output', default='data/burger2/calibration/camera.yaml')
    cmd = commands.add_parser('marker')
    cmd.add_argument('--dictionary', default='DICT_5X5_100')
    cmd.add_argument('--id', type=int, default=23)
    cmd.add_argument('--pixels', type=int, default=600)
    cmd.add_argument('--output', default='data/markers/DICT_5X5_100_id23.png')
    return p


# 영상 스트림을 읽으며 검출 결과를 출력.
def run_stream(args):
    positive(args.timeout, 'timeout')
    if args.show and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('--show requires a desktop display; omit for headless operation')
    if args.source and (args.compressed or getattr(args, 'observation_topic', None)):
        raise ValueError('--compressed/--observation-topic require ROS topic input')
    if args.command == 'capture':
        positive(args.interval, 'interval')
        if args.cols < 2 or args.rows < 2 or args.count < 1:
            raise ValueError('Invalid corner counts or capture count')
        Path(args.output).mkdir(parents=True, exist_ok=True)
    elif args.frames < (1 if args.command == 'probe' else 0):
        raise ValueError('Invalid frame count')
    detector = None
    if args.command == 'detect':
        calibration = load_calibration(args.calibration) if args.calibration else None
        if args.board:
            if args.dictionary is not None or args.target_id is not None or args.marker_m is not None:
                raise ValueError('--board defines marker IDs/dimensions; do not combine with single-marker options')
            detector = BoardDetector(DockingBoard.load(args.board), calibration)
        else:
            detector = Detector(args.dictionary or 'DICT_5X5_100',
                                23 if args.target_id is None else args.target_id,
                                args.marker_m, calibration)
    source = (VideoSource(args.source) if args.source is not None else
              RosSource(args.topic, args.compressed, getattr(args, 'observation_topic', None)))
    count, saved, last_save, first_time = 0, 0, float('-inf'), None
    last_report = float('-inf')
    last_size = None
    try:
        while True:
            frame, metadata = source.read(args.timeout)
            now = time.monotonic()
            if now - metadata['received_monotonic'] > args.timeout:
                raise RuntimeError('Stale input frame')
            size = (frame.shape[1], frame.shape[0])
            if last_size is not None and size != last_size:
                raise ValueError('Camera resolution changed during capture')
            last_size = size
            count += 1
            if first_time is None:
                first_time = now
            view = frame
            corners = None
            if detector is not None:
                observation, view = detector.detect(frame)
                observation.update(metadata)
                observation.update(sequence=count, status='ok')
                if hasattr(source, 'publish'):
                    source.publish(observation)
                if now - last_report >= .5:
                    print(json.dumps(observation, allow_nan=False), flush=True)
                    last_report = now
            elif args.command == 'capture':
                corners = board_corners(frame, args.cols, args.rows)
                view = frame.copy()
                if corners is not None:
                    cv2.drawChessboardCorners(view, (args.cols, args.rows), corners, True)
            key = -1
            if args.show:
                cv2.imshow('burger2 docking camera (q quit / s save)', view)
                key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                if args.command != 'detect':
                    raise RuntimeError('Stopped before requested frames/images were collected')
                break
            if args.command == 'capture':
                save = key == ord('s') if args.show else now - last_save >= args.interval
                if save and corners is not None:
                    # Timestamp names prevent overwriting a prior capture session.
                    path = Path(args.output) / f'checkerboard_{time.time_ns()}.png'
                    write_image(path, frame)  # Save the ORIGINAL, never the corner overlay.
                    saved += 1
                    last_save = now
                    print(f'Saved {saved}/{args.count}: {path}', flush=True)
                if saved >= args.count:
                    break
            elif args.frames and count >= args.frames:
                if args.command == 'probe':
                    write_image(args.output, frame)
                    result = {'frames': count, 'width': size[0], 'height': size[1],
                              'received_fps': ((count - 1) / (now - first_time)
                                               if count > 1 and now > first_time else None),
                              'mean_pixel': float(frame.mean()), 'std_pixel': float(frame.std()),
                              'snapshot': str(Path(args.output).resolve()), **metadata}
                    print(json.dumps(result, indent=2))
                break
    finally:
        if detector is not None and hasattr(source, 'publish'):
            # Consumers must ALSO enforce their own timestamp watchdog.
            target = ({'mode': 'four_marker_board', 'target_ids': detector.board.ids,
                       'alignment_ready': False} if args.board else
                      {'target_id': detector.target_id})
            source.publish({**target, 'detected': False,
                            'pose_valid': False, 'status': 'stopped',
                            'received_monotonic': time.monotonic()})
        source.close()
        if args.show:
            cv2.destroyAllWindows()


# 명령 실행.
def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == 'marker':
            write_image(args.output, make_marker(args.dictionary, args.id, args.pixels))
            print(f'Saved {args.output}; measure the printed BLACK square, excluding white margin.')
        elif args.command == 'calibrate':
            paths = sorted(p for p in Path(args.images).iterdir()
                           if p.suffix.lower() in ('.png', '.jpg', '.jpeg', '.bmp'))
            data = calibrate(paths, args.cols, args.rows, args.square_m,
                             args.camera_name, args.min_views)
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(yaml.safe_dump(data, sort_keys=False))
            np.save(output.parent / 'calibration_matrix.npy',
                    np.array(data['camera_matrix']['data']).reshape(3, 3))
            np.save(output.parent / 'distortion_coefficients.npy',
                    np.array(data['distortion_coefficients']['data']).reshape(1, -1))
            print(f"Saved {output}; RMS={data['rms_px']:.4f}px, views={len(data['accepted_images'])}")
            if data['rms_px'] > 1.0:
                print('Check per-view errors, focus, and board geometry: RMS exceeds 1px.')
        else:
            run_stream(args)
        return 0
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, cv2.error) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
