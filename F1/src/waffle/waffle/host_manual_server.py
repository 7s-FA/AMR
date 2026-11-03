"""ROS adapter for F1 manual commands; opening/subscribing never moves hardware."""
import json
import time
import uuid

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from robot3_interfaces.action import Waffle
from waffle.shared_data import Shared_data
from waffle.host_manual_core import ManualControl, COMMANDS

control = ManualControl(Shared_data, Twist)


class WaffleActionServer(Node):
    def __init__(self):
        super().__init__('data', namespace='F1')
        self.reservation = None
        self.publisher = self.create_publisher(String, '/F1/manual/status', 10)
        self.subscription = self.create_subscription(String, '/F1/manual/command', self.receive, 10)
        self.timer = self.create_timer(0.5, self.report)
        self._action_server = ActionServer(
            self, Waffle, '/F1/data', callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback)
        self.get_logger().info('F1 manual control ready: latched stop + state + existing Waffle Action')

    def report(self):
        self.publisher.publish(String(data=json.dumps(control.snapshot(), ensure_ascii=False)))

    def receive(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                return
            control.receive(request)
            self.report()
        except (ValueError, TypeError) as exc:
            self.get_logger().warning('Invalid F1 manual request: ' + str(exc))

    def goal_callback(self, request):
        with control.lock:
            if request.command == 'EMER_STOP':
                control.stop()
                return GoalResponse.ACCEPT
            if request.command == 'RESTART':
                return GoalResponse.ACCEPT
            if (request.command not in COMMANDS or control.stopped or control.busy()
                    or self.reservation is not None or not control.stationary()):
                return GoalResponse.REJECT
            self.reservation = control.generation
            return GoalResponse.ACCEPT

    def cancel_callback(self, handle):
        control.stop('Action 취소 요청')
        return CancelResponse.ACCEPT

    def execute_callback(self, handle):
        request_id, command = uuid.uuid4().hex, handle.request.command
        result = Waffle.Result()
        try:
            with control.lock:
                if command in COMMANDS:
                    generation, self.reservation = self.reservation, None
                    if generation != control.generation:
                        raise ValueError('시작 전에 정지 요청을 받았습니다.')
                event = control.submit(command, request_id)
            while event['status'] == 'running':
                if not rclpy.ok():
                    control.stop('ROS 종료')
                time.sleep(0.1)
                with control.lock:
                    event = next((e for e in reversed(control.events) if e['id'] == request_id), event)
            result.success = event['status'] == 'success'
            result.message = 'Success' if result.success and command in COMMANDS else event['message']
        except Exception as exc:
            result.success, result.message = False, str(exc)
        if handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()
