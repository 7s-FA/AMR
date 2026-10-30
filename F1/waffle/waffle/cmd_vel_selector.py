import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


class CmdVelSelector(Node):
    def __init__(self):
        super().__init__("cmd_vel_selector")

        # 제어권 요청을 받기 전까지 정지
        self.mode = "STOP"
        self.last_command = None
        self.last_received = 0.0
        self.command_timeout = 0.3

        # 선택기가 최종 구동 명령을 발행
        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # NAV 속도는 충돌 감시를 통과한 값만 수신
        self.nav_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_nav_filtered",
            self.on_nav_velocity,
            10,
        )
        self.dock_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_dock",
            self.on_dock_velocity,
            10,
        )

        self.control_service = self.create_service(
            SetBool,
            "/docking_control",
            self.on_control_request,
        )

        self.backup_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_backup",
            self.on_backup_velocity,
            10,
        )

        self.backup_service = self.create_service(
            SetBool,
            "/backup_control",
            self.on_backup_control,
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_selected,
        )

        self.get_logger().info("속도 선택기 준비: STOP")

    def on_backup_control(self, request, response):
        self.last_command = None
        self.last_received = 0.0
        self.mode = "BACKUP" if request.data else "STOP"

        self.publisher.publish(Twist())

        response.success = True
        response.message = f"control mode: {self.mode}"
        return response


    def on_backup_velocity(self, msg):
        if self.mode != "BACKUP":
            return

        self.last_command = msg
        self.last_received = time.monotonic()

    def on_control_request(self, request, response):
        # 전환 전에 받은 명령은 재사용하지 않음
        self.last_command = None
        self.last_received = 0.0

        self.mode = "DOCK" if request.data else "NAV"

        self.publisher.publish(Twist())

        response.success = True
        response.message = f"control mode: {self.mode}"

        self.get_logger().info(response.message)
        return response

    def on_nav_velocity(self, msg):
        if self.mode != "NAV":
            return

        self.last_command = msg
        self.last_received = time.monotonic()

    def on_dock_velocity(self, msg):
        if self.mode != "DOCK":
            return

        self.last_command = msg
        self.last_received = time.monotonic()

    def publish_selected(self):
        command_expired = (
            time.monotonic() - self.last_received
            > self.command_timeout
        )

        if self.last_command is None or command_expired:
            self.publisher.publish(Twist())
            return

        self.publisher.publish(self.last_command)