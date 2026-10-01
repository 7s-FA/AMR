"""Burger1 hardware with private TF topics and prefixed URDF/odom frame IDs."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    bringup = get_package_share_directory('turtlebot3_bringup')
    return LaunchDescription([GroupAction([
        SetRemap(src='/tf', dst='/burger1/tf'),
        SetRemap(src='/tf_static', dst='/burger1/tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup + '/launch/robot.launch.py'),
            launch_arguments={'namespace': 'burger1'}.items()),
    ])])
