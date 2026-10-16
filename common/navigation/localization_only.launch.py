# ========================================================================
# 역할: 위치추정만 띄우는 launch (map_server + AMCL + lifecycle_manager_localization). 속도 명령 없음.
# 실행: <로봇>-localization.service → launch_localization.sh → ros2 launch localization_only.launch.py
# 참고: 처음 위치는 자동 추측하지 않는다(set_initial_pose=False). check_ready.py 가 확인된 주차 자세만 넣는다.
# ========================================================================
"""Only map_server, AMCL and lifecycle manager; never publishes velocity."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction,OpaqueFunction,RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushROSNamespace,SetParameter
from ament_index_python.packages import get_package_share_directory
import os,tempfile,yaml

# Nav2 설정에서 AMCL·map_server 부분을 꺼내 임시 파라미터 파일을 만들고 노드 3개를 구성.
def prepare(context):
    robot=LaunchConfiguration('robot').perform(context)
    params=yaml.safe_load(open(LaunchConfiguration('params_file').perform(context)))
    # The first session needs a real initial pose, never a silently guessed parking pose.
    params['amcl']['ros__parameters']['set_initial_pose']=False
    params['amcl']['ros__parameters']['always_reset_initial_pose']=False
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(params,f,sort_keys=False);generated=f.name
    # launch 종료 시 임시 파라미터 파일 삭제.
    def cleanup(_context):
        if os.path.exists(generated):os.unlink(generated)
        return []
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        GroupAction([SetParameter('bond_timeout',30.0),PushROSNamespace(robot),IncludeLaunchDescription(
            PythonLaunchDescriptionSource(get_package_share_directory('nav2_bringup')+'/launch/localization_launch.py'),
            launch_arguments={'namespace':robot,'params_file':generated,'map':LaunchConfiguration('map'),
                              'use_sim_time':'false','autostart':'true','use_composition':'False','use_respawn':'False'}.items())])]

# launch 인자(robot, params_file, map) 선언 후 prepare 실행.
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot'),DeclareLaunchArgument('params_file'),
                              DeclareLaunchArgument('map'),OpaqueFunction(function=prepare)])
