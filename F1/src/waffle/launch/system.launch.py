import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    nav2_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/ubuntu/turtlebot3_ws/src/waffle/factory_map.yaml',
            description='지도 YAML 파일의 절대 경로',
        ),
        DeclareLaunchArgument(
            'nav_params',
            default_value='/home/ubuntu/turtlebot3_ws/src/waffle/config/nav2_params.yaml',
            description='Waffle용 Nav2 파라미터 파일의 절대 경로'
        ),

        # 로봇 드라이버·센서 등 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'robot.launch.py')
            ),
        ),

        # 지도 서버·위치 추정·경로 계획·주행 제어 등 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('nav_params'),
                'use_sim_time': 'False',
                'autostart': 'True',
                'slam': 'False',
            }.items(),
        ),

        Node(
            package='waffle',
            executable='main',
            name='waffle_main',
            output='screen',
        ),

         Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            output='screen',
            parameters=[
                '/home/ubuntu/turtlebot3_ws/src/waffle/config/scan_filter.yaml',
                {'use_sim_time': False},
            ],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered'),
            ],
        ),
    ])