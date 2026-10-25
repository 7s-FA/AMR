import math
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from robot3_interfaces.action import Waffle

from waffle.shared_data import Shared_data


class WaffleActionServer(Node):
    def __init__(self):
        super().__init__('data', namespace='F1')

        self._busy = False

        self._action_server = ActionServer(
            self,
            Waffle,
            '/F1/data',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
        )

        self.get_logger().info(
            'Host Action Server ready: /F1/data '
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received goal: command={goal_request.command}, '
            f'cmd_val={goal_request.cmd_val}'
        )

        if self._busy:
            self.get_logger().warn('작업 중이므로 요청을 거절합니다.')
            return GoalResponse.REJECT

        if not goal_request.command.strip():
            self.get_logger().warn('빈 명령을 거절합니다.')
            return GoalResponse.REJECT

        Shared_data.navigator_index = goal_request.command
        Shared_data.navigator_start = True

        self._busy = True
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = Waffle.Result()

        try:
            self.get_logger().info(
                f'통신 시험 시작: {goal_handle.request.command}'
            )

            feedback = Waffle.Feedback()



            # 아래 코드 처럼 feedback data 를 뿌려줘야함... 지금은 우선 X

            # for i in range(10):
            #     feedback.robot_x = float(i)
            #     feedback.robot_y = float(i)
            #     feedback.robot_theta = 'front' if i < 5 else 'back'
            #     feedback.message = f'MOCK: communication test {i + 1}/10'

            #     goal_handle.publish_feedback(feedback)
            #     time.sleep(1.0)

            while not Shared_data.navigator_end:
                if not rclpy.ok():
                    result.success = False
                    result.message = "ROS 종료로 작업 대기를 중단했습니다."

                    if goal_handle.is_active:
                        goal_handle.abort()

                    return result

                time.sleep(0.1)
            
            Shared_data.navigator_end = False

            if  Shared_data.navigator_end_msg == "Success":
                result.success = True
                goal_handle.succeed()

            else:
                result.success = False
                goal_handle.abort()
            result.message = Shared_data.navigator_end_msg

            
            self.get_logger().info(result.message)

        except Exception as exc:
            result.success = False
            result.message = f'Communication test failed: {exc}'

            if goal_handle.is_active:
                goal_handle.abort()

            self.get_logger().error(result.message)

        finally:
            self._busy = False

        return result

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    server = WaffleActionServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()