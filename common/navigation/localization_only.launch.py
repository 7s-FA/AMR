"""Only map_server, AMCL and lifecycle manager; never publishes velocity."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction,OpaqueFunction,RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushROSNamespace,SetParameter
from ament_index_python.packages import get_package_share_directory
import os,tempfile,yaml

def prepare(context):
    robot=LaunchConfiguration('robot').perform(context)
    params=yaml.safe_load(open(LaunchConfiguration('params_file').perform(context)))
    # The first session needs a real initial pose, never a silently guessed parking pose.
    params['amcl']['ros__parameters']['set_initial_pose']=False
    params['amcl']['ros__parameters']['always_reset_initial_pose']=False
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(params,f,sort_keys=False);generated=f.name
    def cleanup(_context):
        if os.path.exists(generated):os.unlink(generated)
        return []
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        GroupAction([SetParameter('bond_timeout',10.0),PushROSNamespace(robot),IncludeLaunchDescription(
            PythonLaunchDescriptionSource(get_package_share_directory('nav2_bringup')+'/launch/localization_launch.py'),
            launch_arguments={'namespace':robot,'params_file':generated,'map':LaunchConfiguration('map'),
                              'use_sim_time':'false','autostart':'true','use_composition':'False','use_respawn':'False'}.items())])]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot'),DeclareLaunchArgument('params_file'),
                              DeclareLaunchArgument('map'),OpaqueFunction(function=prepare)])
