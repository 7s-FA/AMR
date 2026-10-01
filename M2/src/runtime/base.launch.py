"""Burger2 hardware with private TF topics and prefixed URDF/odom frame IDs."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    bringup = get_package_share_directory('turtlebot3_bringup')
    # A failed OpenCR/LDS child must not leave the base service falsely active.
    # Shut down the complete bringup; ready can then start a fresh base.
    stop_on_exit = RegisterEventHandler(OnProcessExit(
        on_exit=lambda event, context: [EmitEvent(event=Shutdown(
            reason="Burger2 base child exited: " + event.process_name))]))
    return LaunchDescription([stop_on_exit, GroupAction([
        SetRemap(src='/tf', dst='/burger2/tf'),
        SetRemap(src='/tf_static', dst='/burger2/tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup + '/launch/robot.launch.py'),
            launch_arguments={'namespace': 'burger2'}.items()),
    ])])
