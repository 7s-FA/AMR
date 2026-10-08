"""Onboard Nav2 under /burger2; the host starts RViz separately."""
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
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='burger2_nav2_',
                                     delete=False, encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        generated = stream.name

    def cleanup(_context):
        if os.path.exists(generated):
            os.unlink(generated)
        return []

    nav2 = get_package_share_directory('nav2_bringup')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        GroupAction([
            # The bundled Nav2 docking server is not used by ArUco/IR docking.
            # Its declared publisher must never share the real motor topic.
            SetRemap(src='docking_server:cmd_vel',dst='cmd_vel_nav_unused'),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share + '/launch/lean_bringup_launch.py'),
            launch_arguments={
                'namespace': 'burger2', 'use_namespace': 'true',
                'use_localization': 'False',
                'map': LaunchConfiguration('map'),
                'params_file': generated,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items())]),
    ]


def generate_launch_description():
    share = get_package_share_directory('waffle_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=share + '/maps/factory_map.yaml'),
        DeclareLaunchArgument('params_file', default_value=share + '/config/nav2_burger2_params.yaml'),
        DeclareLaunchArgument('waypoints_file', default_value=share + '/config/waypoints_burger2.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=prepare),
        Node(package='rviz2', executable='rviz2', name='rviz2', namespace='burger2',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', share + '/rviz/burger2_navigation.rviz'],
             remappings=[('/tf', '/burger2/tf'), ('/tf_static', '/burger2/tf_static')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
