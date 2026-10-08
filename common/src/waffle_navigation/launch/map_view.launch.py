import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('waffle_navigation'))
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=str(package_share / 'rviz' / 'map_building.rviz'),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='waffle_slam_rviz',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])
