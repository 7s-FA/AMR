import math
import queue
import threading
import time
import cv2
import numpy as np
import os
import select
import subprocess
import rclpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from waffle.docking_config import load_params
from waffle.shared_data import Shared_data
from waffle.board_pose import BoardPose

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, limit):
    return max(-limit, min(limit, value))


class DockingMotion(Node):
    def __init__(self):
        super().__init__("waffle_docking_motion")


        share = Path(get_package_share_directory("waffle"))
        self.p = load_params(share / "config" / "docking_params.yaml", {},)
        self.target = self.p["target_cm"] / 100.0 # cm -> m

        # base_footprint 기준 카메라 광학 중심: x 앞쪽+, y 왼쪽+
        self.camera_x = self.p["camera_forward_mm"] / 1000.0 # mm -> m
        self.camera_y = self.p["camera_left_mm"] / 1000.0 # mm -> m

        self.init_camera(share / "calibration")

        self.publisher = self.create_publisher(Twist, "/cmd_vel_dock", 10)       # 도킹 중 로봇에 이동/정지 속도 명령을 보냄 #정제 수정
        self.subscription = self.create_subscription(                       # 로봇의 오도메트리 위치·속도 정보를 받아 on_odom()에서 처리
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )

        self.lock = threading.Lock()
        self.commands = queue.Queue()

        self.pose = None
        self.pose_time = 0.0
        self.pose_reason = "not_received"
        self.detected_ids = ()
        self.last_good_pose = None
        self.last_good_pose_time = 0.0
        self.last_good_odom = None
        self.marker_nudge_count = 0
        self.marker_nudge_origin = None
        self.frame_time = 0.0
        self.odom = None
        self.odom_time = 0.0

        self.state = "WAIT"
        self.started_at = 0.0
        self.stable_since = None
        self.reference = None
        self.target_travel = 0.0
        self.progress_time = 0.0
        self.progress_distance = 0.0
        self.stop_started = 0.0
        self.stop_stable_since = None
        self.last_report = 0.0
        self.resume_state = None
        self.vision_wait_since = None
        self.check_started = 0.0
        self.last_command = (0.0, 0.0)

        self.prepare_started = None
        self.stop_reason= ""
        self.alignment_retry_count = 0
        self.retry_reference = None
        self.retry_resume_state = "VISION_APPROACH"

        self.camera_stop = threading.Event()
        self.camera_thread = None
        self.camera_error = None

        self.timer = self.create_timer(
            self.p["control_period_s"], self.run
        )

    def init_camera(self, calibration_dir):
        self.width = 640
        self.height = 480

        matrix = np.load(
            calibration_dir / "calibration_matrix.npy"
        )
        distortion = np.load(
            calibration_dir / "distortion_coefficients.npy"
        )

        with np.load(
            calibration_dir / "calibration_details.npz"
        ) as details:
            if tuple(details["image_size"]) != (
                self.width,
                self.height,
            ):
                raise RuntimeError("보정 해상도가 640×480이 아닙니다.")

        self.board = BoardPose(
            marker_m=self.p["marker_mm"] / 1000.0,
            gap_m=self.p["gap_mm"] / 1000.0,
            matrix=matrix,
            distortion=distortion,
        )

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )
        self.detector_params.cornerRefinementWinSize = 3 

    def start_camera(self):
        if (
            self.camera_thread is not None
            and self.camera_thread.is_alive()
        ):
            raise RuntimeError(
                "이전 카메라 작업이 종료 중입니다. 잠시 후 다시 요청하세요."
            )

        # 이전 도킹의 관측값을 새 시작에 사용하지 않음
        with self.lock:
            self.pose = None
            self.pose_time = 0.0
            self.pose_reason = "not_received"
            self.frame_time = 0.0
            self.camera_error = None

            self.detected_ids = ()
            self.last_good_pose = None
            self.last_good_odom = None
            self.marker_nudge_count = 0

        self.camera_stop.clear()

        self.camera_thread = threading.Thread(
            target=self.camera_loop,
            daemon=True,
        )
        self.camera_thread.start()


    def stop_camera(self):
        self.camera_stop.set()

    def camera_loop(self):
        process = None

        try:
            process = subprocess.Popen(
                [
                    "rpicam-vid",
                    "--nopreview",
                    "--timeout", "0",
                    "--mode", "1640:1232:10:P",
                    "--width", str(self.width),
                    "--height", str(self.height),
                    "--framerate", "10",
                    "--codec", "mjpeg",
                    "--output", "-",
                ],
                stdout=subprocess.PIPE,
                bufsize=0,
            )

            buffer = bytearray()

            while not self.camera_stop.is_set():
                # 영상이 없더라도 주기적으로 종료 요청 확인
                readable, _, _ = select.select(
                    [process.stdout], [], [], 0.1
                )

                if not readable:
                    continue

                chunk = os.read(process.stdout.fileno(), 65536)

                if not chunk:
                    raise RuntimeError(
                        "카메라 영상 입력이 종료됐습니다."
                    )

                buffer.extend(chunk)

                while not self.camera_stop.is_set():
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

                    if frame.shape[:2] != (
                        self.height,
                        self.width,
                    ):
                        raise RuntimeError(
                            "카메라 촬영 해상도가 보정 해상도와 다릅니다."
                        )

                    # 정상 영상 수신 여부와 마커 검출 여부는 구분
                    self.frame_received()

                    gray = cv2.cvtColor(
                        frame,
                        cv2.COLOR_BGR2GRAY,
                    )

                    corners, ids, _ = cv2.aruco.detectMarkers(
                        gray,
                        self.dictionary,
                        parameters=self.detector_params,
                    )

                    pose, reason = self.board.estimate(
                        corners,
                        ids,
                    )

                    self.update_pose(
                        pose, reason,
                        () if ids is None else tuple(int(i) for i in ids.reshape(-1)),
                    )

        except Exception as exc:
            if not self.camera_stop.is_set():
                # 정지 판단은 ROS 타이머의 run()에서 처리
                with self.lock:
                    self.camera_error = str(exc)

        finally:
            if process is not None:
                try:
                    if process.poll() is None:
                        process.terminate()

                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
                finally:
                    process.stdout.close()

    def on_odom(self, msg):     # 오도메트리 검사/위치/속도 저장
        if (
            msg.header.frame_id != "odom"
            or msg.child_frame_id != "base_footprint"
        ):
            self.odom = None
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        if not all(math.isfinite(x) for x in (
            p.x, p.y, q.x, q.y, q.z, q.w, v, w
        )):
            self.odom = None
            return

        # 오래된 메시지를 현재 오도메트리로 사용하지 않음
        stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if not -0.1 <= ros_now - stamp <= self.p["odom_stamp_timeout_s"]:
            self.odom = None
            return

        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-6:
            self.odom = None
            return

        qx, qy, qz, qw = (
            q.x / norm, q.y / norm, q.z / norm, q.w / norm
        )
        yaw = math.atan2(
            2 * (qw*qz + qx*qy),
            1 - 2 * (qy*qy + qz*qz),
        )

        self.odom = (p.x, p.y, yaw, v, w)
        self.odom_time = time.monotonic()

    def update_pose(self, pose, reason="unknown", detected_ids=()):
        with self.lock:
            self.pose = None if pose is None else pose.copy()
            self.pose_time = time.monotonic()
            self.pose_reason = "ok" if pose is not None else reason
            self.detected_ids = tuple(sorted(detected_ids))
            if pose is not None:
                self.last_good_pose = pose.copy()
                self.last_good_pose_time = self.pose_time
                self.last_good_odom = (
                    self.odom if self.pose_time - self.odom_time <= self.p["odom_timeout_s"]
                    else None
                )

    def frame_received(self):
        with self.lock:
            self.frame_time = time.monotonic()

    def request(self, command):     # 시작/정지 요청을 큐에 넣음
        self.commands.put(command)

    def publish_velocity(self, linear, angular):  # 이동/정지 속도 명령 발행
        if not Shared_data.docking_control_acquired:
            linear = 0.0
            angular = 0.0

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.publisher.publish(msg)
        self.last_command = (float(linear), float(angular))

    def stop(self, reason):
        self.stop_reason = reason
        self.publish_velocity(0.0, 0.0)
        self.state = "STOP"
        self.stable_since = None
        self.get_logger().error(f"[STOP] {reason}")

    def aligned(self, pose, preparation=False):     # 거리/좌우/각도 정렬 판정
        lower = self.p["prep_min_m"] if preparation else self.p["align_min_m"]
        upper = self.p["prep_max_m"] if preparation else self.p["align_max_m"]
        return (
            lower <= pose["plane_distance_m"] <= upper
            and self.on_axis(pose)
        )

    def on_axis(self, pose):        # 거리/좌우/각도 정렬 판정
        return (
            abs(pose["x_m"]) <= self.p["align_lateral_m"]
            and abs(pose["heading_error_deg"]) <= self.p["align_yaw_deg"]
        )

    def stationary(self):       # 오도메트리 기준 정지 판정
        return (
            abs(self.odom[3]) < self.p["stationary_speed_mps"]
            and abs(self.odom[4]) < self.p["stationary_yaw_rate_rps"]
        )

    def enter(self, state, now):        # 도킹 단계 전환
        self.publish_velocity(0.0, 0.0)
        self.state = state
        self.stable_since = None
        self.check_started = now
        print(f"[{state}]", flush=True)

    def begin_alignment_retry(self, now, resume_state="VISION_APPROACH"):
        if self.alignment_retry_count >= self.p["alignment_retry_max_count"]:
            self.stop("alignment retry limit reached")
            return
        self.alignment_retry_count += 1
        self.retry_reference = None
        self.retry_resume_state = resume_state
        self.enter("RETRY_STOP", now)

    def control_alignment_retry(self, pose, now):
        if not Shared_data.docking_control_acquired:
            self.stop("docking control lost during alignment retry")
            return
        if pose is not None and pose["plane_distance_m"] < self.p["too_close_m"]:
            self.stop("too close during alignment retry")
            return

        if self.state != "RETRY_STOP":
            x0, y0, yaw0 = self.retry_reference
            x, y, yaw, _, _ = self.odom
            dx, dy = x - x0, y - y0
            backward = -(dx * math.cos(yaw0) + dy * math.sin(yaw0))
            lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            yaw_error = wrap(yaw - yaw0)
            target = self.p["alignment_retry_distance_m"]

            if (
                abs(lateral) > self.p["straight_lateral_m"]
                or abs(yaw_error) > math.radians(self.p["straight_yaw_deg"])
                or backward < -self.p["reverse_tolerance_m"]
                or backward > target + self.p["overshoot_m"]
            ):
                self.stop("backoff path deviation/odom jump")
                return

            if self.state == "BACKOFF":
                remaining = target - backward
                if remaining <= self.p["stop_remaining_m"]:
                    self.enter("BACKOFF_STOP", now)
                    return

                if backward > self.progress_distance + self.p["progress_step_m"]:
                    self.progress_distance = backward
                    self.progress_time = now
                elif now - self.progress_time > self.p["progress_timeout_s"]:
                    self.stop("no backward progress")
                    return

                speed = min(
                    self.p["alignment_retry_speed_mps"],
                    max(
                        self.p["straight_min_speed_mps"],
                        self.p["straight_distance_gain"] * remaining,
                    ),
                )
                self.publish_velocity(-speed, 0.0)
                return

        # RETRY_STOP / BACKOFF_STOP: 정지가 확인된 뒤 다음 단계로 이동.
        self.publish_velocity(0.0, 0.0)
        if now - self.check_started > self.p["stop_timeout_s"]:
            self.stop("retry stop confirmation timeout")
            return
        if not self.stationary():
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
        if now - self.stable_since < self.p["stop_stable_s"]:
            return

        if self.state == "RETRY_STOP":
            self.retry_reference = self.odom[:3]
            self.progress_time = now
            self.progress_distance = 0.0
            self.enter("BACKOFF", now)
        else:
            self.retry_reference = None
            self.wait_for_vision(self.retry_resume_state, now, "alignment retry")

    def single_marker_missing(self, reason, detected_ids):
        missing = tuple(i for i in (0, 1, 2, 3) if i not in detected_ids)
        return (
            len(detected_ids) == 3
            and len(missing) == 1
            and reason == f"missing_id_{missing[0]}"
        )

    def begin_marker_nudge(self, now):
        """최근 정상 관측이 있는 마커 1개 누락에 설정 거리만큼 접근."""
        with self.lock:
            previous = self.last_good_pose
            previous_odom = self.last_good_odom
            eligible = (
                self.single_marker_missing(self.pose_reason, self.detected_ids)
                and now - self.pose_time <= self.p["vision_timeout_s"]
                and previous is not None
                and previous_odom is not None
                and now - self.last_good_pose_time <= 3.0
            )
        if (not eligible
                or self.marker_nudge_count >= self.p["marker_recovery_max_attempts"]
                or not Shared_data.docking_control_acquired):
            return False
        x, y, yaw, _, _ = self.odom
        x0, y0, yaw0, _, _ = previous_odom
        displacement = math.hypot(x - x0, y - y0)
        # 설정 거리만큼 접근 후에도 준비 정렬 하한보다 멀어야 함.
        if (
            previous["plane_distance_m"] - displacement
            - self.p["marker_recovery_distance_m"] <= self.p["prep_min_m"]
            or abs(previous["heading_error_deg"]) > self.p["line_max_board_bearing_deg"]
            or abs(wrap(yaw - yaw0)) > math.radians(self.p["straight_yaw_deg"])
        ):
            return False
        self.marker_nudge_count += 1
        self.marker_nudge_origin = self.odom[:3]
        self.get_logger().info(
            f"마커 재검출: 최대 {self.p['marker_recovery_distance_m'] * 100:g}cm "
            f"저속 전진 ({self.marker_nudge_count}/{self.p['marker_recovery_max_attempts']})"
        )
        self.enter("MARKER_NUDGE_STOP", now)
        return True

    def control_marker_nudge(self, pose, valid, detected_ids, reason, now):
        if not Shared_data.docking_control_acquired:
            self.stop("marker recovery control lost")
            return
        if not valid and not (
            self.single_marker_missing(reason, detected_ids)
            and now - self.pose_time <= self.p["vision_timeout_s"]
        ):
            self.stop("marker recovery lost remaining markers")
            return
        if valid and pose["plane_distance_m"] < self.p["too_close_m"]:
            self.stop("marker recovery too close")
            return

        if self.marker_nudge_origin is not None:
            x0, y0, yaw0 = self.marker_nudge_origin
            x, y, yaw, _, _ = self.odom
            dx, dy = x - x0, y - y0
            forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
            lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            if (
                abs(lateral) > self.p["straight_lateral_m"]
                or abs(wrap(yaw - yaw0)) > math.radians(self.p["straight_yaw_deg"])
                or forward < -self.p["reverse_tolerance_m"]
                or forward > self.p["marker_recovery_distance_m"] + 0.01
            ):
                self.stop("marker recovery path deviation/odom jump")
                return

        if self.state == "MARKER_NUDGE":
            if valid or forward >= self.p["marker_recovery_distance_m"] - self.p["stop_remaining_m"]:
                self.enter("MARKER_NUDGE_SETTLE", now)
            # 고정 속도 0.01m/s에 이동거리와 2초 여유를 반영.
            elif now - self.check_started >= self.p["marker_recovery_distance_m"] / 0.01 + 2.0:
                self.stop("marker recovery movement timeout")
            else:
                self.publish_velocity(0.01, 0.0)
            return

        self.publish_velocity(0.0, 0.0)
        if now - self.check_started > self.p["stop_timeout_s"]:
            self.stop("marker recovery stop timeout")
            return
        if not self.stationary():
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
        if now - self.stable_since < self.p["stop_stable_s"]:
            return
        if (self.state == "MARKER_NUDGE_STOP" and not valid
                and forward < self.p["marker_recovery_distance_m"] - self.p["stop_remaining_m"]):
            self.enter("MARKER_NUDGE", now)
        else:
            self.wait_for_vision(self.resume_state, now, "marker forward recovery")

    def wait_for_vision(self, resume_state, now, reason):       # 정지 후 보드 재검출 대기
        self.resume_state = resume_state
        self.vision_wait_since = now
        self.enter("VISION_WAIT", now)
        print(f"[VISION_WAIT] {reason}; resume={resume_state}", flush=True)

    def follow_line(self, pose, preparation):       # 접근 방향/속도 계산
        # 보드 법선과 중심으로 정의한 접근선. 카메라 전방이 수평이라는 전제.
        heading = math.radians(pose["heading_error_deg"])
        # 카메라의 접근선 횡오차. +이면 접근선의 왼쪽에 있음.
        camera_cross = pose["x_m"] * math.cos(heading) - pose["z_m"] * math.sin(heading)
        # 목표 방향에서 카메라가 접근선에 오도록 차체 기준 횡오차 환산.
        base_cross = (
            camera_cross - self.camera_x * math.sin(heading)
            + self.camera_y * (1.0 - math.cos(heading))
        )
        lower = self.p["prep_min_m"] if preparation else self.p["align_min_m"]
        upper = self.p["prep_max_m"] if preparation else self.p["align_max_m"]
        # 회전 후 예상 좌우 오차(base_cross)가 작으면 제자리에서 방향 마무리.
        if lower <= pose["plane_distance_m"] <= upper and abs(base_cross) <= self.p["align_lateral_m"]:
            self.enter("PRETURN" if preparation else "FINALTURN", time.monotonic())
            return base_cross, -heading
        # 고정 주시 거리로 근접 시 방향 계산의 급격한 변화를 방지.
        steering = wrap(-heading - math.atan2(base_cross, self.p["line_lookahead_m"]))
        # 회전 후에도 보드 중심이 시야 중앙 범위 안에 남도록 조향 제한.
        bearing = math.atan2(pose["x_m"], pose["z_m"])
        limit = math.radians(self.p["line_max_board_bearing_deg"])
        steering = max(-limit - bearing, min(limit - bearing, steering))
        # 좌우·방향이 이미 허용 범위 안이면 회전 없이 접근.
        if self.on_axis(pose):
            steering = 0.0
        angular = clip(
            self.p["line_heading_gain"] * steering,
            self.p["align_max_yaw_rate_rps"],
        )
        goal = self.p["prep_goal_m"] if preparation else self.p["align_goal_m"]
        slow_stop = self.p["prep_min_m"] if preparation else self.p["approach_stop_m"]
        linear = min(
            self.p["align_max_speed_mps"],
            self.p["align_distance_gain"] * max(0.0, pose["plane_distance_m"] - goal),
            self.p["approach_slow_gain"] * max(0.0, pose["plane_distance_m"] - slow_stop),
        )
        if abs(steering) > math.radians(self.p["line_turn_only_deg"]):
            linear = 0.0
        else:
            # 범위/근접 중단 검사를 통과한 이동에만 드라이버 최소 단위 적용.
            linear = min(self.p["align_max_speed_mps"], max(self.p["align_min_speed_mps"], linear))
        self.publish_velocity(linear, angular)
        return base_cross, steering

    def report_visual(self, pose, now):     # 진행 상태 출력
        if now - self.last_report < 0.5:
            return
        preparation = self.state in ("PREALIGN", "PRETURN", "PRECHECK")
        lower = self.p["prep_min_m"] if preparation else self.p["align_min_m"]
        upper = self.p["prep_max_m"] if preparation else self.p["align_max_m"]
        print(
            f"[{self.state}] d={pose['plane_distance_m']:.3f}m "
            f"x={pose['x_m']*1000:+.1f}mm yaw={pose['heading_error_deg']:+.2f}deg "
            f"distance_ok={lower <= pose['plane_distance_m'] <= upper} "
            f"lateral_ok={abs(pose['x_m']) <= self.p['align_lateral_m']} "
            f"yaw_ok={abs(pose['heading_error_deg']) <= self.p['align_yaw_deg']} "
            f"cmd_v={self.last_command[0]:.4f} cmd_w={self.last_command[1]:.4f} "
            f"odom_v={self.odom[3]:.4f} odom_w={self.odom[4]:.4f}",
            flush=True,
        )
        self.last_report = now

    def inputs_ready(self):
        now = time.monotonic()

        with self.lock:
            vision_ready = (
                self.pose is not None
                and now - self.pose_time <= self.p["vision_timeout_s"]
                and now - self.frame_time <= self.p["camera_timeout_s"]
            )

        odom_ready = (
            self.odom is not None
            and now - self.odom_time <= self.p["odom_timeout_s"]
        )

        return vision_ready and odom_ready

    def run(self):
        try:
            with self.lock:
                camera_error = self.camera_error
                self.camera_error = None

            if camera_error is not None:
                raise RuntimeError(f"camera error: {camera_error}")

            now = time.monotonic()
            command = Shared_data.docking_state

            # 정지 요청
            if command == "docking_stop":
                self.prepare_started = None
                self.request("stop")

            # 시작 요청
            elif command == "docking_start":
                if (
                    self.prepare_started is None
                    and self.state in ("WAIT", "STOP", "DONE")
                ):
                    self.prepare_started = now
                    self.stop_reason = ""
                    self.start_camera()

                # 요청 문자열을 바꿔서 반복 접수 방지
                Shared_data.docking_state = (
                    "docking_ASM_preparing"
                    if self.prepare_started is not None
                    else "docking_ASM_running"
                )

            # 카메라와 odom 준비 대기
            if self.prepare_started is not None:
                if self.inputs_ready():
                    self.prepare_started = None

                    # 기존 제어 로직에 시작 요청 전달
                    self.request("start")
                    self.control_step()

                    # 거리·중복 발행자 등의 기존 검사에서 거절됨
                    if self.state in ("WAIT", "STOP", "DONE"):
                        self.stop("start rejected; check preceding log")

                elif now - self.prepare_started >= 10.0:
                    self.prepare_started = None
                    self.stop("camera/odom preparation timeout")

                else:
                    # 준비가 안 됐으면 다음 타이머 주기에서 다시 확인
                    return

            else:
                # 기존 도킹 제어 한 주기 실행
                self.control_step()

            # 결과와 진행 상태 반영
            if self.state == "DONE":
                Shared_data.docking_state = "docking_ASM_succeed"

            elif self.state == "STOP":
                Shared_data.docking_state = "docking_ASM_stopped"

            elif self.state != "WAIT":
                Shared_data.docking_state = "docking_ASM_running"

        except Exception as exc:
            self.prepare_started = None
            self.stop(f"control error: {exc}")
            Shared_data.docking_state = "docking_ASM_stopped"

        finally:
            if (
                self.prepare_started is None
                and self.state in ("STOP", "DONE")
            ):
                self.stop_camera()

    def control_step(self):
        now = time.monotonic()

        with self.lock:
            pose = self.pose
            pose_time = self.pose_time
            frame_time = self.frame_time
            pose_reason = self.pose_reason
            detected_ids = self.detected_ids

        pending = []
        while True:
            try:
                pending.append(self.commands.get_nowait())
            except queue.Empty:
                break

        # 같은 주기에 시작·정지가 함께 들어오면 정지가 우선
        if "stop" in pending:
            self.stop("operator stop")
            return

        if "start" in pending and self.state in ("WAIT", "STOP", "DONE"):
            if (
                pose is None
                or now - pose_time > self.p["vision_timeout_s"]
                or self.odom is None
                or now - self.odom_time > self.p["odom_timeout_s"]
                or now - frame_time > self.p["camera_timeout_s"]
            ):
                print("[START REJECTED] fresh camera/board/odometry required",
                      flush=True)
                return

            if not Shared_data.docking_control_acquired:
                print(
                    "[START REJECTED] docking control not acquired",
                    flush=True,
                )
                return

            if not max(self.p["start_min_m"], self.p["prep_min_m"]) <= pose["plane_distance_m"] <= self.p["start_max_m"]:
                print(f"[START REJECTED] preparation needs {max(self.p['start_min_m'], self.p['prep_min_m']):.3f}–{self.p['start_max_m']:.3f} m; reposition manually",
                      flush=True)
                return

            self.started_at = now
            self.reference = None
            self.alignment_retry_count = 0
            self.retry_reference = None
            self.wait_for_vision("PREALIGN", now, "initial observation check")

        if self.state == "WAIT":
            return

        if self.state in ("STOP", "DONE"):
            self.publish_velocity(0.0, 0.0)
            return

        if now - frame_time > self.p["camera_timeout_s"]:
            self.stop("camera frame timeout")
            return

        if self.odom is None or now - self.odom_time > self.p["odom_timeout_s"]:
            self.stop("odometry timeout/invalid")
            return

        # if self.count_publishers("/cmd_vel") > 1:
        #     self.stop("another cmd_vel publisher")
        #     return

        if now - self.started_at > self.p["test_timeout_s"]:
            self.stop("test timeout")
            return

        visual_states = ("PREALIGN", "PRETURN", "PRECHECK", "VISION_APPROACH", "FINALTURN", "FINALCHECK")
        valid = pose is not None and now - pose_time <= self.p["vision_timeout_s"]

        if self.state in ("MARKER_NUDGE_STOP", "MARKER_NUDGE", "MARKER_NUDGE_SETTLE"):
            self.control_marker_nudge(pose, valid, detected_ids, pose_reason, now)
            return

        if self.state in ("RETRY_STOP", "BACKOFF", "BACKOFF_STOP"):
            self.control_alignment_retry(pose if valid else None, now)
            return

        if self.state in visual_states and not valid:
            self.wait_for_vision(self.state, now, pose_reason if pose is None else "stale observation")
            return

        if self.state == "VISION_WAIT":
            self.publish_velocity(0.0, 0.0)
            # 복구 중 관측이 또 끊겨도 전체 재검출 대기시간은 늘리지 않음.
            if now - self.vision_wait_since >= self.p["vision_reacquire_timeout_s"]:
                if not valid and self.begin_marker_nudge(now):
                    return
                self.stop(f"vision reacquisition timeout: {pose_reason}")
                return
            if not valid or not self.stationary():
                self.stable_since = None
                return
            if self.stable_since is None:
                self.stable_since = now
            if now - self.stable_since >= self.p["vision_reacquire_stable_s"]:
                self.enter(self.resume_state, now)
            return

        if self.state in visual_states:
            preparation = self.state in ("PREALIGN", "PRETURN", "PRECHECK")
            if pose["plane_distance_m"] < self.p["too_close_m"]:
                self.stop("too close before straight phase")
                return
            if (
                self.state in ("VISION_APPROACH", "FINALTURN", "FINALCHECK")
                and pose["plane_distance_m"] <= self.p["align_min_m"]
                and not self.aligned(pose)
            ):
                self.begin_alignment_retry(now)
                return
            if preparation and pose["plane_distance_m"] < self.p["prep_min_m"]:
                self.begin_alignment_retry(now, "PREALIGN")
                return

            if self.state in ("PREALIGN", "VISION_APPROACH"):
                if self.aligned(pose, preparation):
                    self.enter("PRECHECK" if preparation else "FINALCHECK", now)
                else:
                    lower = self.p["prep_min_m"] if preparation else self.p["align_min_m"]
                    if pose["plane_distance_m"] <= lower:
                        self.stop("alignment corridor exhausted; reposition manually")
                        return
                    self.follow_line(pose, preparation)
                self.report_visual(pose, now)
                return

            if self.state in ("PRETURN", "FINALTURN"):
                if self.aligned(pose, preparation):
                    self.enter("PRECHECK" if preparation else "FINALCHECK", now)
                elif pose["plane_distance_m"] < (
                    self.p["prep_min_m"] if preparation else self.p["align_min_m"]
                ):
                    self.stop("alignment corridor exhausted; reposition manually")
                elif self.on_axis(pose) and pose["plane_distance_m"] > (
                    self.p["prep_max_m"] if preparation else self.p["align_max_m"]
                ):
                    # 회전에 따른 카메라 위치 변화로 거리가 멀어진 경우만 재접근.
                    self.enter("PREALIGN" if preparation else "VISION_APPROACH", now)
                elif now - self.check_started >= self.p["alignment_turn_timeout_s"]:
                    self.stop("in-place alignment timeout; reposition manually")
                else:
                    # 제자리 정렬은 전진 없이 중심을 맞추고 방향 조건도 확인.
                    if abs(pose["x_m"]) > self.p["align_lateral_m"]:
                        error = math.atan2(pose["x_m"], pose["z_m"] + self.camera_x)
                    else:
                        error = math.radians(pose["heading_error_deg"])
                    angular = clip(-self.p["line_heading_gain"] * error, self.p["align_max_yaw_rate_rps"])
                    if angular != 0.0:
                        angular = math.copysign(max(self.p["align_min_yaw_rate_rps"], abs(angular)), angular)
                    self.publish_velocity(0.0, angular)
                if self.state != "STOP":
                    self.report_visual(pose, now)
                return

            # 정렬 조건을 벗어나면 연속 확인 타이머 초기화.
            self.publish_velocity(0.0, 0.0)
            self.report_visual(pose, now)
            if not self.aligned(pose, preparation) or not self.stationary():
                self.stable_since = None
            elif self.stable_since is None:
                self.stable_since = now
            elif now - self.stable_since >= self.p["settle_s"]:
                if preparation:
                    self.enter("VISION_APPROACH", now)
                else:
                    self.reference = self.odom[:3]
                    heading = math.radians(pose["heading_error_deg"])
                    self.target_travel = (pose["plane_distance_m"] - self.target) / math.cos(heading)
                    self.progress_time = now
                    self.progress_distance = 0.0
                    self.enter("STRAIGHT", now)
                    print(f"[STRAIGHT] target travel={self.target_travel:.3f} m; angular=0", flush=True)
                return
            if now - self.check_started >= self.p["alignment_check_timeout_s"]:
                self.begin_alignment_retry(
                    now, "PREALIGN" if preparation else "VISION_APPROACH"
                )
            return

        if self.state == "STRAIGHT":
            x0, y0, yaw0 = self.reference
            x, y, yaw, _, _ = self.odom
            dx, dy = x - x0, y - y0

            forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
            lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            yaw_error = wrap(yaw - yaw0)

            if (
                abs(lateral) > self.p["straight_lateral_m"]
                or abs(yaw_error) > math.radians(self.p["straight_yaw_deg"])
                or forward < -self.p["reverse_tolerance_m"]
                or forward > self.target_travel + self.p["overshoot_m"]
            ):
                self.stop("straight path deviation/odom jump")
                return

            remaining = self.target_travel - forward

            if remaining <= self.p["stop_remaining_m"]:
                self.publish_velocity(0.0, 0.0)
                self.state = "STOPPING"
                self.stop_started = now
                self.stop_stable_since = None
                return

            if forward > self.progress_distance + self.p["progress_step_m"]:
                self.progress_distance = forward
                self.progress_time = now
            elif now - self.progress_time > self.p["progress_timeout_s"]:
                self.stop("no forward progress")
                return

            # 목표·이탈·정체 검사를 통과한 경우에만 최소속도로 직진.
            linear = min(
                self.p["straight_max_speed_mps"],
                max(
                    self.p["straight_min_speed_mps"],
                    self.p["straight_distance_gain"] * remaining,
                ),
            )

            self.publish_velocity(linear, 0.0)

            if now - self.last_report > 0.5:
                print(
                    f"[STRAIGHT] remaining={remaining * 1000:.1f} mm | "
                    f"cmd_v={linear:.4f} m/s | "
                    f"odom_v={self.odom[3]:.4f} m/s",
                    flush=True,
                )
                self.last_report = now
            return

        if self.state == "STOPPING":
            self.publish_velocity(0.0, 0.0)

            stationary = (
                abs(self.odom[3]) < self.p["stationary_speed_mps"]
                and abs(self.odom[4]) < self.p["stationary_yaw_rate_rps"]
            )
            if stationary:
                if self.stop_stable_since is None:
                    self.stop_stable_since = now
                elif now - self.stop_stable_since >= self.p["stop_stable_s"]:
                    self.state = "DONE"

                    # 층 변경은 Navigator가 리니어 상승까지 확인한 뒤 완료한다.
                    if Shared_data.sequence_phase == "FLOOR_WAIT_DOCK":
                        Shared_data.sequence_phase = "FLOOR_DOCK_DONE"
                        print("[DONE] floor-switch docking completed", flush=True)
                        return

                    if Shared_data.docking_place != Shared_data.cmd_WAFFLE_ASM:
                        Shared_data.navigator_end_msg = "Success"
                        Shared_data.navigator_end = True

                    if  Shared_data.docking_place == Shared_data.cmd_WAFFLE_PARK:
                        Shared_data.linear_action = "down"
                    
                    elif Shared_data.docking_place == Shared_data.cmd_WAFFLE_ASM:

                        Shared_data.linear_action = "middle"
                        Shared_data.sequence_phase = "ASM_WAIT_MIDDLE"

                    elif Shared_data.docking_place == Shared_data.cmd_WAFFLE_A_01 or Shared_data.docking_place == Shared_data.cmd_WAFFLE_B_01:
                        Shared_data.storage_state = not Shared_data.storage_state # 정제 추가
                        
                        if  Shared_data.linear_pallet_exist == True:
                            Shared_data.linear_action = "1st_floor_down"
                            Shared_data.linear_pallet_exist = False
                        else:
                            Shared_data.linear_action = "1st_floor_up"
                            Shared_data.linear_pallet_exist = True
                    
                    elif Shared_data.docking_place == Shared_data.cmd_WAFFLE_A_02 or Shared_data.docking_place == Shared_data.cmd_WAFFLE_B_02:
                        Shared_data.storage_state = not Shared_data.storage_state # 정제 추가

                        if  Shared_data.linear_pallet_exist == True:
                            Shared_data.linear_action = "2nd_floor_down"
                            Shared_data.linear_pallet_exist = False
                        else:
                            Shared_data.linear_action = "up"
                            Shared_data.linear_pallet_exist = True

                    else:
                        Shared_data.linear_action = "down" # 정제 추가
                        self.get_logger().info(f"docking_place error : 입력된 docking place는 {Shared_data.docking_place}입니다.")

                    print("[DONE] estimated approach completed",
                          flush=True)
            else:
                self.stop_stable_since = None

            if self.state != "DONE" and now - self.stop_started > self.p["stop_timeout_s"]:
                self.stop("stop confirmation timeout")

    def destroy_node(self): 
        self.timer.cancel()

        try:
            self.publish_velocity(0.0, 0.0)

        finally:
            self.stop_camera()
            try:
                if self.camera_thread is not None:
                    self.camera_thread.join(timeout=5.0)

                    if self.camera_thread.is_alive():
                        self.get_logger().error("카메라 스레드 종료 실패")

            finally:
                super().destroy_node()