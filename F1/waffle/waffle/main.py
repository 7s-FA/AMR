import os
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from waffle.navigator import Navigator
from waffle.linear import Linear
from waffle.docking_motion import DockingMotion
from waffle.cmd_vel_selector import CmdVelSelector
from waffle.host_action_server import WaffleActionServer
from waffle.shared_data import Shared_data

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    velocity_selector = CmdVelSelector()
    waypoint_mover = Navigator()
    linear_mover = Linear()
    docking_mover = DockingMotion()

    host_action_server = WaffleActionServer()

    nodes = [
        velocity_selector,
        waypoint_mover,
        linear_mover,
        docking_mover,
        host_action_server,
    ]

    executor = MultiThreadedExecutor()

    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        waypoint_mover.get_logger().info("종료 요청")

    finally:
        executor.shutdown()

        for node in nodes:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()