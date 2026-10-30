import math
import time
import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose

from nav2_msgs.srv import ManageLifecycleNodes

from waffle.shared_data import Shared_data

from std_srvs.srv import SetBool

from nav2_msgs.msg import SpeedLimit

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

WAYPOINTS_HOME = (0.086, -1.477, -3.1415)
WAYPOINTS_ASM = (1.086, -1.277, 1.4389) # (1.209 -1.249, 1.3089)
WAYPOINTS_WARE_TOP = (0.486, -1.477, -1.5708) # s2
WAYPOINTS_WARE_BOT = (1.086, -1.477, -1.5708) # s1


# WAYPOINTS_HOME = (0.086, -1.577, -3.101)
# WAYPOINTS_ASM = (1.209, -1.239, 1.4389) # (1.209 -1.249, 1.3089)
# WAYPOINTS_WARE_TOP = (0.475, -1.380, -1.5708) # s2
# WAYPOINTS_WARE_BOT = (1.089, -1.380, -1.5708) # s1

DICT_WAYPOINTS_FOR_CMD = {
    Shared_data.cmd_WAFFLE_PARK : WAYPOINTS_HOME,
    Shared_data.cmd_WAFFLE_ASM : WAYPOINTS_ASM,
    Shared_data.cmd_WAFFLE_A_01 : WAYPOINTS_WARE_BOT,
    Shared_data.cmd_WAFFLE_A_02 : WAYPOINTS_WARE_BOT,
    Shared_data.cmd_WAFFLE_B_01 : WAYPOINTS_WARE_TOP,
    Shared_data.cmd_WAFFLE_B_02 : WAYPOINTS_WARE_TOP
}


class Navigator(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='waypoint_mover')

        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            '/waffle/nav_speed_limit',
            10,
        )

        self.get_logger().info("\033[1;32mwaypoint_mover 생성\033[0m")

        initial_pose = self.make_pose(0.086, -1.577, -3.1415)
        self.setInitialPose(initial_pose)

        self.waitUntilNav2Active()
        self.get_logger().info('Nav2 준비 완료')

        Shared_data.navigator_start = False
        Shared_data.navigator_index = ""

        self.operating = False
        self.active_index = None

        self.goal_future = None

        self.nav2_manager = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes'
        )

        self.control_client = self.create_client(
            SetBool,
            "/docking_control",
        )

        self.control_mode = None
        self.control_transition_pending = False
        Shared_data.docking_control_acquired = False

        self.extra_wait_started = None
        self.backup_done = False
        self.departure_from_park = True  # 프로그램은 PARK 위치에서 시작
        self.departure_distance_m = 0.40

        self.backup_phase = "IDLE"
        self.backup_future = None
        self.backup_started = 0.0
        self.backup_origin = None

        self.backup_odom = None
        self.backup_odom_received = 0.0
        self.backup_stop_since = None


        self.backup_pub = self.create_publisher(
            Twist, "/cmd_vel_backup", 10
        )
        self.backup_client = self.create_client(
            SetBool, "/backup_control"
        )
        self.backup_odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.on_backup_odom,
            qos_profile_sensor_data,
        )

        self.control_timer = self.create_timer(0.1, self.run)

    def on_backup_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        values = (p.x, p.y, q.x, q.y, q.z, q.w, v, w)
        if not all(math.isfinite(value) for value in values):
            self.backup_odom = None
            return

        stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        if not -0.1 <= now_ros - stamp <= 0.5:
            self.backup_odom = None
            return

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self.backup_odom = (p.x, p.y, yaw, v, w)
        self.backup_odom_received = time.monotonic()


    def publish_backup(self, speed=0.0):
        msg = Twist()
        msg.linear.x = float(speed)
        self.backup_pub.publish(msg)


    def fail_backup(self, reason):
        self.publish_backup()
        # 실패 후 자동 재시도하지 않음
        self.backup_phase = "FAILED"
        Shared_data.sequence_phase = ""
        self.extra_wait_started = None
        Shared_data.navigator_start = False
        Shared_data.navigator_end_msg = f"Backup failed: {reason}"
        Shared_data.navigator_end = True
        self.get_logger().error(Shared_data.navigator_end_msg)


    def reset_backup(self):
        self.backup_done = False
        self.backup_phase = "IDLE"
        self.backup_future = None
        self.backup_origin = None
        self.backup_stop_since = None

    def run_floor_switch(self):
        index = Shared_data.navigator_index
        phase = Shared_data.sequence_phase
        switches = {
            Shared_data.cmd_WAFFLE_A_01: Shared_data.cmd_WAFFLE_A_02,
            Shared_data.cmd_WAFFLE_A_02: Shared_data.cmd_WAFFLE_A_01,
            Shared_data.cmd_WAFFLE_B_01: Shared_data.cmd_WAFFLE_B_02,
            Shared_data.cmd_WAFFLE_B_02: Shared_data.cmd_WAFFLE_B_01,
        }
        now = time.monotonic()

        if phase == "":
            if (
                self.operating
                or not Shared_data.navigator_start
                or switches.get(Shared_data.previous_command) != index
            ):
                return False
            Shared_data.navigator_start = False
            Shared_data.departure_reset_requested = False
            phase = "FLOOR_WAIT_PREVIOUS"
            Shared_data.sequence_phase = phase
            self.extra_wait_started = now

        if not phase.startswith("FLOOR_"):
            return False

        target = (
            "1st_floor_down"
            if index in (Shared_data.cmd_WAFFLE_A_01, Shared_data.cmd_WAFFLE_B_01)
            else "2nd_floor_down"
        )

        if phase in ("FLOOR_WAIT_PREVIOUS", "FLOOR_WAIT_HEIGHT"):
            ready = Shared_data.linear_action == "stop"
            if phase == "FLOOR_WAIT_HEIGHT":
                ready = ready and Shared_data.linear_state == target
            if not ready:
                if now - self.extra_wait_started > 30.0:
                    Shared_data.linear_action = "stop"
                    self.fail_backup("층 변경 리니어 완료 대기 시간 초과")
                return True
            self.extra_wait_started = now if phase == "FLOOR_WAIT_HEIGHT" else None
            self.reset_backup()
            Shared_data.sequence_phase = (
                "FLOOR_BACKWARD" if phase == "FLOOR_WAIT_PREVIOUS"
                else "FLOOR_DOCK_START"
            )
            return True

        if phase == "FLOOR_BACKWARD":
            if not self.prepare_first_departure(direction=-1.0):
                return True
            Shared_data.linear_action = target
            Shared_data.sequence_phase = "FLOOR_WAIT_HEIGHT"
            self.extra_wait_started = now
            return True

        if phase == "FLOOR_DOCK_START":
            self.publish_backup()
            if now - self.extra_wait_started > 30.0:
                self.fail_backup("층 변경 도킹 제어권 대기 시간 초과")
                return True
            if not self.control_client.service_is_ready():
                return True
            Shared_data.docking_place = index
            Shared_data.sequence_phase = "FLOOR_WAIT_DOCK"
            if not self.select_control(docking=True):
                self.fail_backup("층 변경 도킹 제어권 요청 실패")
            return True

        if phase == "FLOOR_WAIT_DOCK":
            if not Shared_data.docking_control_acquired:
                self.fail_backup("층 변경 도킹 제어권 전환 실패")
            elif Shared_data.docking_state == "docking_ASM_stopped":
                self.fail_backup("층 변경 도킹 실패; 도킹 STOP 로그 확인")
            return True

        if phase == "FLOOR_DOCK_DONE":
            Shared_data.linear_action = (
                "1st_floor_up" if target == "1st_floor_down" else "up"
            )
            Shared_data.sequence_phase = "FLOOR_WAIT_UP"
            self.extra_wait_started = now
            return True

        if phase == "FLOOR_WAIT_UP":
            self.publish_backup()
            up_target = "1st_floor_up" if target == "1st_floor_down" else "up"
            if not (
                Shared_data.linear_state == up_target
                and Shared_data.linear_action == "stop"
            ):
                if now - self.extra_wait_started > 30.0:
                    Shared_data.linear_action = "stop"
                    self.fail_backup("층 변경 후 리니어 상승 완료 대기 시간 초과")
                return True

            self.extra_wait_started = None
            Shared_data.linear_pallet_exist = True
            Shared_data.storage_state = True
            Shared_data.departure_reset_requested = False
            Shared_data.docking_place = index
            Shared_data.sequence_phase = ""
            Shared_data.navigator_end_msg = "Success"
            Shared_data.navigator_end = True
            self.get_logger().info(f"{index}: 도킹 및 {up_target} 완료")
            return True

        return False

    def run_extra_sequence(self):
        phase = Shared_data.sequence_phase

        if phase == "":
            storage_commands = (
                Shared_data.cmd_WAFFLE_A_01,
                Shared_data.cmd_WAFFLE_A_02,
                Shared_data.cmd_WAFFLE_B_01,
                Shared_data.cmd_WAFFLE_B_02,
            )
            if (
                not self.operating
                and Shared_data.navigator_start
                and not self.departure_from_park
                and Shared_data.navigator_index in storage_commands
                and Shared_data.storage_state is True
            ):
                self.reset_backup()
                phase = "STORAGE_FORWARD"
                Shared_data.sequence_phase = phase
            else:
                return True

        if phase in ("ASM_WAIT_MIDDLE", "STORAGE_WAIT_UP"):
            target = "middle" if phase == "ASM_WAIT_MIDDLE" else "up"
            now = time.monotonic()
            if self.extra_wait_started is None:
                self.extra_wait_started = now

            if not (
                Shared_data.linear_state == target
                and Shared_data.linear_action == "stop"
            ):
                if now - self.extra_wait_started > 30.0:
                    Shared_data.linear_action = "stop"
                    self.fail_backup(f"{target} 완료 대기 시간 초과")
                return False

            self.extra_wait_started = None
            self.reset_backup()
            phase = "STORAGE_BACKWARD" if phase == "STORAGE_WAIT_UP" else "ASM_BACKWARD"
            Shared_data.sequence_phase = phase

        if phase in ("ASM_BACKWARD", "STORAGE_FORWARD", "STORAGE_BACKWARD"):
            direction = 1.0 if phase == "STORAGE_FORWARD" else -1.0
            if not self.prepare_first_departure(direction):
                return False

            if phase == "ASM_BACKWARD":
                Shared_data.sequence_phase = ""
                Shared_data.navigator_end_msg = "Success"
                Shared_data.navigator_end = True
            elif phase == "STORAGE_BACKWARD":
                # 창고 복귀용 후진을 마쳤으므로 공통 출발 후진은 생략
                Shared_data.departure_reset_requested = False
                Shared_data.linear_action = "down"  # 추가
                Shared_data.sequence_phase = "STORAGE_READY"
                return True
            else:
                Shared_data.linear_action = "up"
                Shared_data.sequence_phase = "STORAGE_WAIT_UP"
                self.extra_wait_started = time.monotonic()
            return False

        return phase == "STORAGE_READY"

    def prepare_first_departure(self, direction=-1.0, distance_m=0.40):
        """후진 및 정지가 완료됐을 때만 True."""
        if self.backup_done:
            return True

        if self.backup_phase == "FAILED":
            self.fail_backup("이전 후진 실패 상태입니다. 점검 후 재시작하세요.")
            return False

        now = time.monotonic()

        if self.backup_phase == "IDLE":
            self.backup_started = now
            self.backup_phase = "WAIT_CONTROL"

        # 추가 거리는 기존 저속(0.08m/s) 기준으로 제한시간을 늘림.
        timeout_s = 15.0 + max(0.0, distance_m - 0.40) / 0.08
        if now - self.backup_started > timeout_s:
            self.fail_backup("후진 준비 또는 이동 시간 초과")
            return False

        if self.backup_phase == "WAIT_CONTROL":
            if self.backup_future is None:
                if not self.backup_client.service_is_ready():
                    return False

                request = SetBool.Request()
                request.data = True
                self.backup_future = self.backup_client.call_async(request)
                return False

            if not self.backup_future.done():
                return False

            try:
                response = self.backup_future.result()
                if response is None or not response.success:
                    self.fail_backup("제어권 획득 실패")
                    return False
            except Exception as exc:
                self.fail_backup(str(exc))
                return False

            self.control_mode = "BACKUP"
            Shared_data.docking_control_acquired = False
            self.backup_phase = "WAIT_ODOM"

        odom_fresh = (
            self.backup_odom is not None
            and now - self.backup_odom_received <= 0.5
        )

        if not odom_fresh:
            self.publish_backup()
            if self.backup_phase != "WAIT_ODOM":
                self.fail_backup("오도메트리 수신 중단")
            return False

        x, y, yaw, v, w = self.backup_odom

        if self.backup_phase == "WAIT_ODOM":
            self.backup_origin = (x, y, yaw)
            self.backup_phase = "MOVING"

        if self.backup_phase == "MOVING":
            x0, y0, yaw0 = self.backup_origin
            dx, dy = x - x0, y - y0

            # 시작 시 로봇 전방 축의 반대 방향으로 이동한 거리
            travelled = direction * (dx * math.cos(yaw0) + dy * math.sin(yaw0))
            lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            yaw_error = math.atan2(
                math.sin(yaw - yaw0),
                math.cos(yaw - yaw0),
            )

            

            remaining = distance_m - travelled

            if remaining > 0.0:
                if  Shared_data.linear_pallet_exist == True:
                    speed = 0.08
                else:
                    speed = 0.25
                    
                self.publish_backup(direction * speed)
                return False

            self.publish_backup()
            self.backup_phase = "STOPPING"
            self.backup_stop_since = None
            return False

        if self.backup_phase == "STOPPING":
            self.publish_backup()

            # 실제 정지 상태를 0.3초 확인한 뒤 Nav2로 넘어감
            if abs(v) < 0.01 and abs(w) < 0.03:
                if self.backup_stop_since is None:
                    self.backup_stop_since = now

                if now - self.backup_stop_since >= 0.3:
                    self.backup_done = True
                    self.backup_phase = "DONE"
                    self.get_logger().info(f"{distance_m * 100:.0f}cm 이동 및 정지 완료")
                    return True
            else:
                self.backup_stop_since = None

        return False

    def publish_nav_speed_limit(self, speed):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = float(speed)
        self.speed_limit_pub.publish(msg)

    def on_nav_feedback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining

        if not math.isfinite(distance) or distance < 0.0:
            return

        if distance <= 0.20:
            speed = 0.08
        elif distance < 0.50:
            # 50cm → 20cm 구간에서 0.20 → 0.08m/s로 감소
            speed = 0.08 + (distance - 0.20) * 0.4
        else:
            speed = 0.20

        self.publish_nav_speed_limit(speed)

    def select_control(self, docking):
        if self.control_transition_pending:
            return False

        if not self.control_client.service_is_ready():
            return False

        request = SetBool.Request()
        request.data = docking

        self.control_transition_pending = True
        self.control_mode = None
        Shared_data.docking_control_acquired = False

        try:
            future = self.control_client.call_async(request)
            future.add_done_callback(
                lambda completed: self.on_control_selected(
                    completed, docking
                )
            )
        except Exception as exc:
            self.control_transition_pending = False
            Shared_data.navigator_start = False
            self.get_logger().error(
                f"제어권 전환 요청 실패: {exc}"
            )
            return False

        return True


    def on_control_selected(self, future, docking):
        try:
            response = future.result()

            if response is None or not response.success:
                Shared_data.navigator_start = False
                self.get_logger().error("제어권 전환 실패")
                return

            self.control_mode = "DOCK" if docking else "NAV"
            Shared_data.docking_control_acquired = docking

            self.get_logger().info(
                f"제어권 전환 완료: {self.control_mode}"
            )

            if docking:
                Shared_data.docking_state = "docking_start"

        except Exception as exc:
            self.control_mode = None
            Shared_data.docking_control_acquired = False
            Shared_data.navigator_start = False
            self.get_logger().error(
                f"제어권 전환 응답 오류: {exc}"
            )

        finally:
            self.control_transition_pending = False

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def run(self):
        if not self.initial_pose_received:
            return
            
        if self.control_transition_pending:
            return

        if self.run_floor_switch():
            return

        if not self.run_extra_sequence():
            return

        # 1. 대기 중이면 새로운 시작 요청 확인
        if not self.operating:
            if Shared_data.navigator_start is not True:
                return  

            if Shared_data.docking_state in (
                "docking_start",
                "docking_preparing",
                "docking_running",
                "docking_stop",
            ):
                return

            # 추가: NAV 제어권을 얻은 뒤 기존 이동 처리 진행
            index = Shared_data.navigator_index
            if index not in DICT_WAYPOINTS_FOR_CMD:
                Shared_data.navigator_start = False
                Shared_data.navigator_end_msg = f"Unknown command: {index}"
                Shared_data.navigator_end = True
                return

            # 새 명령마다 한 번만 출발 후진 상태를 초기화
            if Shared_data.departure_reset_requested:
                self.reset_backup()
                Shared_data.departure_reset_requested = False
                self.departure_distance_m = (
                    1.00
                    if self.departure_from_park and index in (
                        Shared_data.cmd_WAFFLE_A_01,
                        Shared_data.cmd_WAFFLE_A_02,
                        Shared_data.cmd_WAFFLE_ASM,
                    )
                    else 0.40
                )

                # PARK 출발 시 리니어 하강 지시
                if index == Shared_data.cmd_WAFFLE_PARK:
                    Shared_data.linear_action = "down"

                self.get_logger().info(
                    f"{index}: 출발 전 {self.departure_distance_m * 100:.0f}cm 후진 시작"
                )

            # 후진 및 정지가 끝날 때까지 Nav2 목표 전송을 보류
            if not self.prepare_first_departure(
                direction=-1.0, distance_m=self.departure_distance_m
            ):
                return

            if self.control_mode != "NAV":
                self.select_control(docking=False)
                return

            index = Shared_data.navigator_index
            Shared_data.navigator_start = False
            self.departure_from_park = False
            Shared_data.sequence_phase = ""

             # 창고에서 ASM으로 출발할 때, 후진 완료 후 리니어 하강
            if (
                index == Shared_data.cmd_WAFFLE_ASM
                and Shared_data.docking_place in (
                    Shared_data.cmd_WAFFLE_A_01,
                    Shared_data.cmd_WAFFLE_A_02,
                    Shared_data.cmd_WAFFLE_B_01,
                    Shared_data.cmd_WAFFLE_B_02,
                )
            ):
                Shared_data.linear_action = "down"


            goal = NavigateToPose.Goal()
            # 이 아래는 기존 코드 그대로
            goal.pose = self.make_pose(*DICT_WAYPOINTS_FOR_CMD[index])

            self.active_index = index

            # 새 목표 시작 시 이전 접근 속도 제한을 갱신
            self.publish_nav_speed_limit(0.20)

            self.goal_future = (
                self.nav_to_pose_client.send_goal_async(
                    goal,
                    feedback_callback=self.on_nav_feedback,
                )
            )
            self.operating = True

            self.get_logger().info(f'{index} 목표 이동 요청')
            return

        # 2. 목표 수락 여부 확인
        if self.goal_future is not None:
            if not self.goal_future.done():
                return

            goal_handle = self.goal_future.result()
            self.goal_future = None

            if goal_handle is None or not goal_handle.accepted:
                self.operating = False
                self.get_logger().error('목표가 거절되었습니다.')
                Shared_data.navigator_end_msg = "Fail"
                Shared_data.navigator_end = True
                return

            self.result_future = goal_handle.get_result_async()
            return

        # 3. 이동 완료 여부 확인
        if self.result_future is None or not self.result_future.done():
            return

        self.status = self.result_future.result().status
        self.result_future = None
        self.operating = False

        if self.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info(f'{self.active_index} 목표 도착')

            if  self.active_index == Shared_data.cmd_WAFFLE_PARK:
                self.departure_from_park = True
                Shared_data.navigator_end_msg = "Success"
                Shared_data.navigator_end = True

                # down 을 기본 상태 및 초기화 상태로 봄
                Shared_data.docking_place = Shared_data.cmd_WAFFLE_PARK

            else:

                Shared_data.docking_state = "docking_start"

                Shared_data.docking_place = Shared_data.linear_action

                if not self.select_control(docking=True):
                    self.get_logger().error(
                        "도킹 제어권 요청 실패. 도킹을 시작하지 않습니다."
                    )

            Shared_data.docking_place = self.active_index

            if  self.active_index == Shared_data.cmd_WAFFLE_PARK:
                Shared_data.linear_action = "down"
            
            elif self.active_index == Shared_data.cmd_WAFFLE_ASM:
                Shared_data.linear_action = "up"
            
            elif self.active_index == Shared_data.cmd_WAFFLE_A_01:
                if  Shared_data.linear_pallet_exist == True:
                    Shared_data.linear_action = "1st_floor_up"
                else:
                    Shared_data.linear_action = "1st_floor_down"
            
            elif self.active_index == Shared_data.cmd_WAFFLE_A_02:
                if  Shared_data.linear_pallet_exist == True:
                    Shared_data.linear_action = "up"
                else:
                    Shared_data.linear_action = "2nd_floor_down"

            elif self.active_index == Shared_data.cmd_WAFFLE_B_01:
                if  Shared_data.linear_pallet_exist == True:
                    Shared_data.linear_action = "1st_floor_up"
                else:
                    Shared_data.linear_action = "1st_floor_down"
                    
            elif self.active_index == Shared_data.cmd_WAFFLE_B_02:
                if  Shared_data.linear_pallet_exist == True:
                    Shared_data.linear_action = "up"
                else:
                    Shared_data.linear_action = "2nd_floor_down"
        else:
            Shared_data.navigator_end_msg = "Fail"
            Shared_data.navigator_end = True
            self.get_logger().warning('이동 실패 또는 취소.')