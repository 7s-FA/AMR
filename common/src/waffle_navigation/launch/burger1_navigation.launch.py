# ========================================================================
# 역할: burger1 Nav2 실행 launch (우리 코드). 로봇 설정 + 도착 조정값을 합친 임시 파라미터로 lean_bringup_launch 를 실행한다.
#       Nav2 도킹 서버 출력은 실제 모터 토픽과 섞이지 않게 cmd_vel_nav_unused 로 돌린다. RViz 는 관제 PC 에서 따로.
# 실행: burger1-nav2.service → launch_nav2.sh → ros2 launch waffle_navigation burger1_navigation.launch.py
# ========================================================================
"""Onboard Nav2 under /burger1; the host starts RViz separately."""
import importlib.util
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


# 파라미터 파일 + arrival_tuning 병합 → 임시 YAML → lean_bringup_launch 포함 (종료 시 임시 파일 삭제).
def prepare(context):
    share = get_package_share_directory('waffle_navigation')
    helper_path = share + '/config/arrival_tuning.py'
    spec = importlib.util.spec_from_file_location('arrival_tuning', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    # 웨이포인트와 같은 파일의 도착 튜닝을 Nav2의 base params 위에 겹쳐 쓴다.
    tuning = helper.load_tuning(LaunchConfiguration('waypoints_file').perform(context))
    source = LaunchConfiguration('params_file').perform(context)
    with open(source, encoding='utf-8') as stream:
        params = helper.merge_params(yaml.safe_load(stream), tuning)
    helper.validate_collision_frames(params)
    params['collision_monitor']['ros__parameters']['cmd_vel_out_topic']='cmd_vel_nav_out'
    # 원본 설정은 덮어쓰지 않고 실행 종료 시 임시 파일만 정리한다.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='burger1_nav2_',
                                     delete=False, encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        generated = stream.name

    # 임시 파라미터 파일 삭제.
    def cleanup(_context):
        if os.path.exists(generated):
            os.unlink(generated)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        GroupAction([
            # The bundled Nav2 docking server is not used by ArUco/IR docking.
            # Its declared publisher must never share the real motor topic.
            SetRemap(src='docking_server:cmd_vel',dst='cmd_vel_nav_unused'),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share + '/launch/lean_bringup_launch.py'),
            launch_arguments={
                'namespace': 'burger1', 'use_namespace': 'true',
                'use_localization': 'False',
                'map': LaunchConfiguration('map'),
                'params_file': generated,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items())]),
    ]


# launch 인자 선언(params_file, waypoints_file, use_rviz 등) 후 prepare 실행.
def generate_launch_description():
    share = get_package_share_directory('waffle_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=share + '/maps/factory_map.yaml'),
        DeclareLaunchArgument('params_file', default_value=share + '/config/nav2_burger1_params.yaml'),
        DeclareLaunchArgument('waypoints_file', default_value=share + '/config/waypoints_burger1.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=prepare),
        Node(package='rviz2', executable='rviz2', name='rviz2', namespace='burger1',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', share + '/rviz/burger1_navigation.rviz'],
             remappings=[('/tf', '/burger1/tf'), ('/tf_static', '/burger1/tf_static')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
