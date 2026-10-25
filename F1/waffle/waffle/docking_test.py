from pathlib import Path
import argparse
import select
import socket
import subprocess

import cv2
import numpy as np

import time
import rclpy

from docking_motion import DockingMotion
from docking_config import load_params
from board_pose import BoardPose

BASE = Path(__file__).resolve().parent
CALIBRATION = BASE / "calibration"

WIDTH, HEIGHT = 640, 480
TARGET_IDS = {0, 1, 2, 3}
VIDEO_PORT = 5000
CONTROL_PORT = 5001

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=BASE / "docking_params.yaml")
    for option in ("marker-mm", "gap-mm", "target-cm", "camera-forward-mm", "camera-left-mm"):
        parser.add_argument("--" + option, type=float, default=None)
    args = parser.parse_args()
    keys = ("marker_mm", "gap_mm", "target_cm", "camera_forward_mm", "camera_left_mm")
    try:
        params = load_params(args.config, {key: getattr(args, key) for key in keys})
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    for key in keys:
        setattr(args, key, params[key])
    print(f"설정: {args.config.resolve()}", flush=True)
    print(f"정렬 거리: {params['align_min_m']:.3f}~{params['align_max_m']:.3f} m, "
          f"좌우: {params['align_lateral_m']*1000:.1f} mm, "
          f"각도: {params['align_yaw_deg']:.1f} deg, 최종 간격: {args.target_cm:.1f} cm", flush=True)

    marker_m = args.marker_mm / 1000.0

    matrix = np.load(CALIBRATION / "calibration_matrix.npy")
    distortion = np.load(
        CALIBRATION / "distortion_coefficients.npy"
    )

    with np.load(CALIBRATION / "calibration_details.npz") as details:
        if tuple(details["image_size"]) != (WIDTH, HEIGHT):
            raise RuntimeError("보정 해상도가 640×480이 아닙니다.")

    board = BoardPose(
        marker_m=marker_m,
        gap_m=args.gap_mm / 1000.0,
        matrix=matrix,
        distortion=distortion,
    )

    last_report_time = 0.0

    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", CONTROL_PORT))
        server.listen(1)

        print("PC의 camera_viewer.py 연결을 기다립니다.", flush=True)
        control, address = server.accept()

        with control, socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        ) as video:
            destination = (address[0], VIDEO_PORT)
            control.sendall(b"Docking: g=start, SPACE=stop, q=quit\n")

            process = subprocess.Popen(
                [
                    "rpicam-vid",
                    "--nopreview",
                    "--timeout", "0",
                    "--mode", "1640:1232:10:P",
                    "--width", str(WIDTH),
                    "--height", str(HEIGHT),
                    "--framerate", "10",
                    "--codec", "mjpeg",
                    "--output", "-",
                ],
                stdout=subprocess.PIPE,
            )

            buffer = bytearray()
            commands = ""
            running = True
            motion = None

            try:
                rclpy.init()
                motion = DockingMotion(
                    target_m=args.target_cm / 100.0,
                    camera_x_m=args.camera_forward_mm / 1000.0,
                    camera_y_m=args.camera_left_mm / 1000.0,
                    params=params,
                )

                while running and rclpy.ok():

                    chunk = process.stdout.read1(65536)
                    if not chunk:
                        raise RuntimeError("카메라 영상 입력이 종료됐습니다.")

                    buffer.extend(chunk)

                    while running:
                        start = buffer.find(b"\xff\xd8")
                        if start < 0:
                            break

                        end = buffer.find(b"\xff\xd9", start + 2)
                        if end < 0:
                            break

                        jpeg = bytes(buffer[start:end + 2])
                        del buffer[:end + 2]

                        frame = cv2.imdecode(
                            np.frombuffer(jpeg, dtype=np.uint8),
                            cv2.IMREAD_COLOR,
                        )
                        if frame is None:
                            continue

                        if frame.shape[:2] != (HEIGHT, WIDTH):
                            raise RuntimeError("촬영 해상도가 다릅니다.")

                        # 기존 PC viewer에서 보내는 키 명령 처리
                        if select.select([control], [], [], 0)[0]:
                            data = control.recv(1024)
                            if not data:
                                motion.request("stop")
                                running = False
                                break
                            commands += data.decode()

                        while "\n" in commands:
                            key, commands = commands.split("\n", 1)
                            if key == "h":
                                motion.heartbeat()
                            elif key == "g":
                                motion.request("start")
                            elif key == " ":
                                motion.request("stop")
                            elif key == "q":
                                motion.request("stop")
                                running = False
                                break

                        if not running:
                            break

                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        corners, ids, _ = cv2.aruco.detectMarkers(
                            gray,
                            dictionary,
                            parameters=parameters,
                        )
                        pose, reason = board.estimate(corners, ids)
                        motion.update_pose(pose, reason)

                        # 영상에는 숫자를 쌓지 않고 터미널에만 출력
                        now = time.monotonic()
                        if now - last_report_time >= 0.5:
                            if pose is None:
                                print(f"보드 계산 불가: {reason}", flush=True)
                            else:
                                print(
                                    f"평면 거리={pose['plane_distance_m'] * 100:.1f} cm | "
                                    f"좌우={pose['x_m'] * 1000:+.1f} mm | "
                                    f"깊이 Z={pose['z_m'] * 100:.1f} cm | "
                                    f"방향={pose['heading_error_deg']:+.1f} deg | "
                                    f"RMS={pose['reprojection_rms_px']:.2f} px",
                                    flush=True,
                                )

                            last_report_time = now

                        view = frame.copy()

                        # 검출된 목표 마커의 테두리와 ID만 표시
                        if ids is not None:
                            for points, marker_id in zip(corners, ids.flatten()):
                                marker_id = int(marker_id)
                                if marker_id not in TARGET_IDS:
                                    continue

                                cv2.aruco.drawDetectedMarkers(
                                    view,
                                    [points],
                                    np.array([[marker_id]], dtype=np.int32),
                                )

                        # 네 마커로 계산한 가상 보드 중심
                        if pose is not None:
                            u, v = pose["center_uv"]
                            target = (
                                int(round(float(u))),
                                int(round(float(v))),
                            )

                            cv2.circle(view, target, 6, (0, 0, 0), -1)
                            cv2.circle(view, target, 4, (0, 255, 0), -1)

                        # 화면 가운데에 빨간 점 고정
                        # 마커 검출 여부와 관계없이 항상 표시
                        center = (WIDTH // 2, HEIGHT // 2)

                        # 검은 테두리를 추가해 밝은 배경에서도 잘 보이도록 표시
                        cv2.circle(view, center, 6, (0, 0, 0), -1)
                        cv2.circle(view, center, 4, (0, 0, 255), -1)

                        for quality in (60, 40, 20):
                            ok, encoded = cv2.imencode(
                                ".jpg", view,
                                [cv2.IMWRITE_JPEG_QUALITY, quality],
                            )
                            if ok and encoded.nbytes <= 60000:
                                video.sendto(
                                    encoded.tobytes(), destination
                                )
                                break

            finally:
                try:
                    if motion is not None:
                        motion.close()
                    elif rclpy.ok():
                        rclpy.shutdown()
                finally:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    process.stdout.close()


if __name__ == "__main__":
    main()