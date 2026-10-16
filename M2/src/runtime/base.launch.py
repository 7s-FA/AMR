# ========================================================================
# 역할: burger2 본체 실행 (TurtleBot3 bringup: OpenCR 모터·odom, LDS-02 라이다, robot_state_publisher).
#       TF 를 /burger2/tf 로 분리해 다른 로봇과 섞이지 않게 한다.
# 실행: burger2-base.service → start_base.sh → ros2 launch base.launch.py
# ========================================================================
"""Burger2 hardware with private TF topics and prefixed URDF/odom frame IDs."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


# TurtleBot3 robot.launch.py 를 네임스페이스·TF 재매핑과 함께 포함.
def generate_launch_description():
    bringup = get_package_share_directory('turtlebot3_bringup')
    return LaunchDescription([GroupAction([
        SetRemap(src='/tf', dst='/burger2/tf'),
        SetRemap(src='/tf_static', dst='/burger2/tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup + '/launch/robot.launch.py'),
            launch_arguments={'namespace': 'burger2'}.items()),
    ])])
