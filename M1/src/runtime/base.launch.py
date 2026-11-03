# ========================================================================
# 역할: 버거3(STM32F411 + micro-ROS) 본체 launch. start_base.sh 가 실행 (micro-ROS Agent·STM32 리셋은 그 스크립트가 먼저).
#   - stm32_adapter (팀원 브리지: /stm32/* ↔ /burger1/*). 속도 명령은 cmd_throttle.py 가 15Hz 이하로 줄여 cmd_vel_stm32 로
#   - wheel_odom.py: 이동은 바퀴 엔코더, 회전은 IMU 로 odom·TF 계산 (브리지 stm32_odometry 는 IMU 짝맞춤 때문에 3Hz 라 안 씀)
#   - robot_state_publisher (turtlebot3_state_publisher, frame_prefix burger1/)
#   - LDS-02 라이다 ld08_driver (/burger1/scan, frame burger1/base_scan)
#   - stm32_compat.py: OpenCR 이 하던 sensor_state(토크·엔코더)·motor_power 서비스 대체
#   TF 는 /burger1/tf 로 분리한다.
# ========================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap

NS = os.environ.get('AMR_ROBOT', 'burger1')
HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_bringup')
    return LaunchDescription([GroupAction([
        SetRemap(src='/tf', dst=f'/{NS}/tf'),
        SetRemap(src='/tf_static', dst=f'/{NS}/tf_static'),
        PushRosNamespace(NS),
        Node(package='burger_stm32_bridge', executable='stm32_adapter', name='stm32_adapter', output='screen',
             parameters=[{'frame_prefix': NS, 'transport_namespace': '/stm32', 'enable_stamped_cmd_vel': True}],
             remappings=[('cmd_vel', 'cmd_vel_stm32')]),   # cmd_throttle.py 를 거쳐 받음
        Node(executable='python3', arguments=[os.path.join(HERE, 'cmd_throttle.py')], name='cmd_throttle',
             output='screen'),   # /burger1/cmd_vel(50Hz) → cmd_vel_stm32(최대 15Hz)
        Node(executable='python3', arguments=[os.path.join(HERE, 'wheel_odom.py')], name='wheel_odom',
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'turtlebot3_state_publisher.launch.py')),
            launch_arguments={'namespace': NS, 'use_sim_time': 'false'}.items()),
        # 버거3의 ld08.launch.py 는 /burger3/scan 으로 고쳐져 있어 쓰지 않고 노드를 직접 띄운다.
        Node(package='ld08_driver', executable='ld08_driver', name='ld08_driver',
             output='screen', parameters=[{'frame_id': 'base_scan', 'namespace': NS}]),
        Node(executable='python3', arguments=[os.path.join(HERE, 'stm32_compat.py')], name='stm32_compat',
             output='screen', parameters=[{'transport_namespace': '/stm32'}]),
    ])])
