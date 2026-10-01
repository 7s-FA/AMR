"""Burger Nav2 + RViz with per-waypoint forward/reverse controllers."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('waffle_navigation')
    tb3_share = get_package_share_directory('turtlebot3_navigation2')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=share + '/maps/factory_map.yaml'),
        DeclareLaunchArgument('params_file', default_value=share + '/config/nav2_burger_params.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_share + '/launch/navigation2.launch.py'),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
    ])
