#!/usr/bin/env python3
"""One-folder entry point. Setup never starts robot services or sends motion goals."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
from materialize import ROOT, materialize


def configuration(robot, runtime=None):
    runtime = Path(runtime).resolve() if runtime else ROOT/'.runtime'/robot
    config = json.loads((runtime/'runtime.json').read_text())
    if config.get('robot_id') != robot or (config.get('linked') and config.get('repository') != str(ROOT)) or config['workspace'] != str(runtime/'final_robot_ws'):
        raise RuntimeError('Copied/moved runtime: run configure --archive-runtime, then build and install-services --replace while stopped')
    return runtime, config


def environment(config):
    return dict(os.environ, AMR_WORKSPACE=config['workspace'], AMR_CAMERA=config['camera'],
                AMR_ROBOT=config['runtime_robot'], BURGER_PROJECT_ROOT=config['workspace'],
                BURGER_NAV2_BASE_DIR=config['camera'], ROS_DOMAIN_ID='40', TURTLEBOT3_MODEL='burger', LDS_MODEL='LDS-02',
                PYTHONPATH=os.pathsep.join([config['navigation'],config['camera'],os.environ.get('PYTHONPATH','')]))


def stopped(robot):
    name = 'burger'+robot[-1]
    result = subprocess.run(['systemctl','--user','list-units','--all','--no-legend','--plain',
                             name+'-*.service',robot.lower()+'-action.service'], check=True, capture_output=True, text=True)
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) >= 4 and fields[2] not in ('inactive','failed'):
            raise RuntimeError('Stop the existing robot services before replacing runtime/units: '+fields[0])
    running = subprocess.run(['pgrep','-u',str(os.getuid()),'-f',
        '[/]nav2_waypoints|[d]ocking_node.py|[t]urtlebot3_ros|[l]d08_driver|[a]mr_mission.action_server'], capture_output=True)
    if running.returncode == 0:
        raise RuntimeError('A robot process is still running; stop it before switching deployments')
    if running.returncode != 1:
        raise RuntimeError('Could not verify robot processes')


def install_units(robot, runtime, config, replace=False):
    dest = Path.home()/'.config/systemd/user'
    units = list(Path(config['units']).glob('*.service'))
    differing = [u for u in units if (dest/u.name).exists() and (dest/u.name).read_bytes() != u.read_bytes()]
    if differing and not replace:
        raise RuntimeError('Existing service paths differ. While stopped, run install-services --replace; old units will be backed up.')
    if differing:
        stopped(robot)
        backup = runtime/'service_backups'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        backup.mkdir(parents=True)
        for unit in differing:
            (backup/unit.name).write_bytes((dest/unit.name).read_bytes())
        print('Previous units saved:', backup)
    dest.mkdir(parents=True, exist_ok=True)
    for unit in units:
        (dest/unit.name).write_bytes(unit.read_bytes())
    subprocess.run(['systemctl','--user','daemon-reload'],check=True)
    print('Services registered. Nothing started; no goals sent.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    target = p.add_mutually_exclusive_group(required=True)
    target.add_argument('--robot', choices=['M1','M2'], help='Use the selected profile in AMR/.runtime')
    target.add_argument('--runtime', type=Path, help='Use an existing runtime (original CLI compatibility)')
    p.add_argument('command', choices=['configure','setup','build','camera-build','install-services','doctor',
        'host-ready','ready','parked','status','stop','mode','mat','asm','rest','park','action'])
    p.add_argument('args', nargs=argparse.REMAINDER)
    a = p.parse_args()
    if a.runtime:
        if a.command in ('configure','setup'):
            p.error('configure/setup requires --robot M1 or --robot M2')
        runtime = a.runtime.resolve()
        a.robot = json.loads((runtime/'runtime.json').read_text())['robot_id']
        if a.robot not in ('M1','M2'):
            p.error('Unsupported runtime robot')
    else:
        runtime = ROOT/'.runtime'/a.robot
    if a.command in ('configure','setup'):
        if a.args not in ([], ['--archive-runtime']):
            p.error('configure/setup accepts only --archive-runtime')
        if a.args and runtime.exists():
            stopped(a.robot)
            archive = runtime.with_name(a.robot+'_saved_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            runtime.rename(archive)
            print('Previous runtime preserved:', archive)
        if not runtime.exists():
            materialize(a.robot, runtime, linked=True)
        runtime, config = configuration(a.robot, runtime)
        print('Selected profile:', a.robot, '\nRuntime:', runtime)
        if a.command == 'configure': return 0
        subprocess.run(['/bin/bash',str(ROOT/'scripts/build.sh'),str(runtime)], check=True, env=environment(config))
        subprocess.run(['/bin/bash',str(ROOT/'scripts/build_camera.sh'),str(runtime)], check=True, env=environment(config))
        install_units(a.robot, runtime, config)
        return 0
    runtime, config = configuration(a.robot, runtime)
    env = environment(config)
    nav = Path(config['navigation'])
    if a.command in ('build','camera-build'):
        if a.args: p.error('Build commands take no extra arguments')
        script = 'build.sh' if a.command == 'build' else 'build_camera.sh'
        return subprocess.call(['/bin/bash',str(ROOT/'scripts'/script),str(runtime)],env=env)
    if a.command == 'install-services':
        if a.args not in ([], ['--replace']): p.error('Use install-services [--replace]')
        install_units(a.robot,runtime,config,bool(a.args));return 0
    if a.command == 'doctor':
        return subprocess.call([sys.executable,str(ROOT/'scripts/doctor.py'),str(runtime)],env=env)
    if a.command == 'host-ready':
        if a.args not in ([], ['--parked']): p.error('Use host-ready [--parked]')
        identity = a.robot+'-'+datetime.now().strftime('%Y%m%dT%H%M%S%f')
        command = ['python3',str(nav/'host_prepare.py'),config['runtime_robot'],'--request-id',identity,*a.args]
    elif a.command == 'action':
        return subprocess.call(['/bin/bash',str(runtime/'start_action.sh'),*a.args],env=env)
    else:
        args = [a.command,*a.args]
        if a.command in ('mat','asm','rest','park'):
            args = ['submit',a.command,'--source','manual',*a.args]
        command = ['python3',str(nav/'operation.py'),*args]
    return subprocess.call(['/bin/bash','-ec','source "$1/nav_env.bash"; shift; exec "$@"','amr',str(nav),*command], env=env)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print('AMR:', exc, file=sys.stderr)
        raise SystemExit(1)
