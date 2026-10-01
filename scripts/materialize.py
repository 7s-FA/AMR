#!/usr/bin/env python3
"""Build an isolated on-robot runtime from shared source and one robot profile."""
import argparse,hashlib,json,os,shutil
import yaml
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def materialize(robot,output):
    manifest=json.loads((ROOT/robot/'runtime_manifest.json').read_text())
    out=Path(output).resolve()
    if out.exists():raise ValueError('Output already exists; choose a new directory: '+str(out))
    if any(c.isspace() for c in str(out)):raise ValueError('Runtime path must not contain whitespace')
    profile=yaml.safe_load((ROOT/robot/'config/robot.yaml').read_text())
    if profile['robot_id']!=robot or profile['runtime_namespace']!=manifest['runtime_robot']:raise ValueError('Profile identity mismatch')
    n=manifest['runtime_robot']; workspace=out/'final_robot_ws';camera=out/manifest['camera_directory']
    targets={'workspace':workspace,'camera':camera,'units':out/'units'}
    plan=[]
    for e in manifest['files']:
        source=(ROOT/e['source']).resolve()
        if not source.is_relative_to(ROOT):raise ValueError('Invalid source path')
        if hashlib.sha256(source.read_bytes()).hexdigest()!=e['sha256']:raise ValueError('Manifest mismatch: '+e['source'])
        kind,rel=e['target'].split('/',1);dest=targets[kind]/rel
        if not dest.resolve().is_relative_to(targets[kind]):raise ValueError('Invalid destination path')
        plan.append((source,dest))
    out.mkdir(parents=True)
    for source,dest in plan:
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
        if source.suffix in ('.sh','.bash','.py','.service','.yaml'):
            s=dest.read_text()
            for old in ['/home/ubuntu/final_robot_ws','%h/final_robot_ws','$HOME/final_robot_ws']:
                s=s.replace(old,str(workspace))
            for old in ['/home/ubuntu/'+manifest['camera_directory'],'%h/'+manifest['camera_directory'],'$HOME/'+manifest['camera_directory']]:
                s=s.replace(old,str(camera))
            dest.write_text(s)
    # All paths named host_ws/handoff below are retained only for runtime compatibility.
    env=workspace/'handoff/pc_env.bash'
    env.write_text(f'''# Robot-only ROS environment generated for {robot}.
source /opt/ros/jazzy/setup.bash
source "$HOME/turtlebot3_ws/install/local_setup.bash"
source "{workspace}/host_ws/install/local_setup.bash"
export BURGER_PROJECT_ROOT="{workspace}" BURGER_NAV2_BASE_DIR="{camera}"
export ROS_DOMAIN_ID=40 TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
''')
    for name in ['amr_mission']:
        shutil.copytree(ROOT/'common/src'/name,workspace/'host_ws/src'/name)
    shutil.copytree(ROOT/'interfaces/host_pkg',workspace/'host_ws/src/host_pkg')
    shutil.copy2(ROOT/'common/navigation/action_gate.py',workspace/'robot'/n/'navigation/action_gate.py')
    nav=workspace/'robot'/n/'navigation'
    (nav/'action_profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    units=out/'units';units.mkdir(exist_ok=True)
    for suffix,script in [('base',camera/'start_base.sh'),('camera',camera/'start_camera.sh'),('nav-control',nav/'nav_control_service.sh')]:
        (units/f'{n}-{suffix}.service').write_text(f'[Unit]\nDescription={robot} {suffix}\n[Service]\nType=simple\nExecStart=/bin/bash {script}\nKillSignal=SIGINT\nTimeoutStopSec=20\nRestart=no\n')
    action=out/'start_action.sh'
    action.write_text(f'''#!/usr/bin/env bash
set -eo pipefail
source "{nav}/nav_env.bash"
exec ros2 run amr_mission action_server --ros-args -p robot_id:={robot} -p runtime_robot:={n} -p navigation_dir:={nav} -p action_name:={profile['action_name']} "$@"
''');action.chmod(0o755)
    (units/f'{robot.lower()}-action.service').write_text(f'[Unit]\nDescription={robot} robot action server\nAfter={n}-motion-owner.service\n[Service]\nType=simple\nExecStart=/bin/bash {action}\nKillSignal=SIGINT\nTimeoutStopSec=25\nRestart=no\n')
    (out/'runtime.json').write_text(json.dumps({'robot_id':robot,'runtime_robot':n,'workspace':str(workspace),'camera':str(camera),'navigation':str(nav),'units':str(units)},indent=2)+'\n')
    return out
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('robot',choices=['M1','M2']);p.add_argument('--output',required=True);a=p.parse_args();print(materialize(a.robot,a.output))
