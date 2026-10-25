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


WAYPOINTS_HOME = (0.086, -1.577, -3.101)
WAYPOINTS_ASM = (1.169, -1.249, 1.3089) # (1.169, -1.249, 1.3089)
WAYPOINTS_WARE_TOP = (0.825, -1.377, -1.5708) # s2
WAYPOINTS_WARE_BOT = (1.089, -1.377, -1.5708) # s1

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

        initial_pose = self.make_pose(0.086, -1.577, -3.101)
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


        self.control_timer = self.create_timer(0.1, self.run)

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
            speed = 0.10
        elif distance < 0.50:
            # 50cm → 20cm 구간에서 0.20 → 0.10m/s로 감소
            speed = 0.10 + (distance - 0.20) * 0.2
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
            if self.control_mode != "NAV":
                self.select_control(docking=False)
                return

            index = Shared_data.navigator_index
            Shared_data.navigator_start = False

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
                Shared_data.linear_action = "" # 1층 높이에 맞기 나와야함
            elif self.active_index == Shared_data.cmd_WAFFLE_A_02:
                Shared_data.linear_action = ""
            elif self.active_index == Shared_data.cmd_WAFFLE_B_01:
                Shared_data.linear_action = "" # 1층 높이에 맞기 나와야함
            elif self.active_index == Shared_data.cmd_WAFFLE_B_02:
                Shared_data.linear_action = ""
            else:
                Shared_data.linear_action = "down"
        else:
            Shared_data.navigator_end_msg = "Fail"
            Shared_data.navigator_end = True
            self.get_logger().warning('이동 실패 또는 취소.')