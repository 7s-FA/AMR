"""Relocation, profile isolation and retry behavior without robot/hardware access."""
import ast
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from materialize import materialize
from robot import environment


@pytest.mark.parametrize('robot', ['M1','M2'])
@pytest.mark.parametrize('linked', [False,True])
def test_runtime_is_complete_relocatable_and_uses_selected_profile(tmp_path,robot,linked):
    out = materialize(robot,tmp_path/robot,linked)
    cfg = json.loads((out/'runtime.json').read_text())
    nav = Path(cfg['navigation'])
    rest_unit = out/'units'/f"{cfg['runtime_robot']}-rest-ready.service"
    assert rest_unit.is_file()
    assert str(nav/'start_rest_ready.sh') in rest_unit.read_text()
    other = 'burger2' if robot=='M1' else 'burger1'
    for name in ['mission_retry.py','retry_ready.py','waypoint_worker.py','waypoint_client.py',
                 'rest_ready_worker.py','host_prepare.py','startup_state.py','ready_parallel.py']:
        assert (nav/name).is_file(), name
        assert (nav/name).is_symlink() == linked
    assert json.loads((nav/'mission_retry.json').read_text())['retries']==3
    for name in ['retry_ready.py','waypoint_worker.py','rest_ready_worker.py']:
        code = (nav/name).read_text()
        assert '/home/ubuntu/' not in code
        assert other not in code
    for p in out.rglob('*'):
        assert not (p.is_symlink() and not p.exists()), p
        if p.suffix == '.py':ast.parse(p.read_text())
        if p.suffix in ('.sh','.bash'):
            subprocess.run(['bash','-n',str(p)],check=True,capture_output=True)
    # Source-discovery must stay at runtime, not resolve into common/navigation.
    code = '''import sys,json
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from operation import Operation
op=Operation(Path(sys.argv[1]))
print(json.dumps([op.robot,str(op.data)]))
'''
    result = subprocess.run([sys.executable,'-c',code,str(nav)],check=True,capture_output=True,text=True,env=environment(cfg))
    assert json.loads(result.stdout)==[cfg['runtime_robot'],str(Path(cfg['workspace'])/'data'/cfg['runtime_robot'])]
    for unit in (out/'units').glob('*.service'):
        text = unit.read_text()
        assert '/home/ubuntu/final_robot_ws/' not in text
        assert '/home/ubuntu/final_robot_camera' not in text
        assert other not in text
        for line in text.splitlines():
            if line.startswith('ExecStart=/bin/bash '):
                entry = Path(line.split(' ',1)[1])
                assert entry.is_file() and entry.is_relative_to(out)


def load_retry():
    spec=importlib.util.spec_from_file_location('retry_for_test',ROOT/'common/navigation/mission_retry.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


def test_retry_stops_and_recovers_before_each_repeat():
    module=load_retry();events=[]
    def fail():events.append('run');raise RuntimeError('failure')
    policy=module.RetryPolicy(3,lambda:events.append('stop'),lambda _:events.append('recover'),lambda *a:None)
    with pytest.raises(module.RetryExhausted):policy.execute('navigation',fail)
    assert events==['run','stop','recover','run','stop','recover','run','stop','recover','run','stop']


def test_emergency_stop_revokes_retry():
    module=load_retry();enabled=[True];events=[]
    def fail():events.append('run');raise RuntimeError('failure')
    def stop():enabled[0]=False
    policy=module.RetryPolicy(3,stop,lambda _:events.append('recover'),lambda *a:None,authorized=lambda:enabled[0])
    with pytest.raises(module.RetryCancelled):policy.execute('navigation',fail)
    assert events==['run']


def test_incomplete_departure_is_never_repeated(tmp_path):
    module=load_retry();path=tmp_path/'progress.json'
    progress=module.RouteProgress(path,[{'x':1,'y':2}],30)
    progress.claim_backup()
    again=module.RouteProgress(path,[{'x':1,'y':2}],30)
    with pytest.raises(RuntimeError):again.claim_backup()


def test_linked_edit_is_a_repository_edit(tmp_path,monkeypatch):
    # Copy a miniature repository fixture so the test cannot edit real sources.
    import shutil
    import materialize as module
    repo=tmp_path/'AMR'
    shutil.copytree(ROOT,repo,ignore=shutil.ignore_patterns('.git','.runtime','.validation','__pycache__'))
    monkeypatch.setattr(module,'ROOT',repo)
    out=module.materialize('M2',repo/'.runtime/M2',True)
    file=out/'final_robot_ws/robot/burger2/navigation/mission_retry.json'
    source=file.resolve();assert source.is_relative_to(repo/'M2')
    file.write_text('{"retries": 2}\n')
    assert json.loads(source.read_text())['retries']==2


@pytest.mark.parametrize('robot', ['M1','M2'])
def test_symlink_entry_imports_runtime_local_modules(tmp_path,robot):
    out = materialize(robot,tmp_path/robot,True)
    cfg = json.loads((out/'runtime.json').read_text())
    camera = Path(cfg['camera'])
    (camera/'runtime_only_probe.py').write_text('VALUE="selected runtime"\n')
    canonical = tmp_path/'canonical.py'
    canonical.write_text('from runtime_only_probe import VALUE\nprint(VALUE)\n')
    entry = camera/'probe.py'
    entry.symlink_to(canonical)
    env = dict(os.environ)
    env.pop('PYTHONPATH',None)
    result = subprocess.run(['bash','-ec','source "$1"; exec "$2" "$3"',
                             'probe',str(out/'runtime.env'),sys.executable,str(entry)],
                            env=env,check=True,capture_output=True,text=True)
    assert result.stdout.strip()=='selected runtime'
    direct = subprocess.run([sys.executable,str(entry)],env=environment(cfg),check=True,capture_output=True,text=True)
    assert direct.stdout.strip()=='selected runtime'


def test_ament_copied_relative_link_is_reanchored(tmp_path):
    from repair_install_links import repair
    source=tmp_path/'src/pkg/scripts/entry.py'
    source.parent.mkdir(parents=True)
    canonical=tmp_path/'canonical.py';canonical.write_text('print(1)\n')
    source.symlink_to(os.path.relpath(canonical,source.parent))
    installed=tmp_path/'install/pkg/lib/pkg/entry'
    installed.parent.mkdir(parents=True)
    installed.symlink_to(os.readlink(source))
    assert not installed.exists()
    repair(tmp_path)
    assert installed.read_text()==canonical.read_text()
    repair(tmp_path)
    assert installed.resolve()==canonical


def test_parked_hands_owned_preparation_lock_to_child(tmp_path,monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR',str(tmp_path))
    spec=importlib.util.spec_from_file_location('operation_test',ROOT/'common/navigation/operation.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nav=tmp_path/'workspace/robot/burger1/navigation'
    nav.mkdir(parents=True)
    guard=ROOT/'common/navigation/mission_guard.sh'
    # The child performs the real admission check, but no robot operations.
    (nav/'confirm_parked.sh').write_text(f'source "{guard}"\nmission_guard burger1 "${{1:-}}"\n')
    op=module.Operation(nav)
    monkeypatch.setattr(op,'busy',lambda **kwargs:False)
    assert op.parked()['localization_ready']


def test_preparation_guard_still_rejects_other_or_invalid_owner(tmp_path,monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR',str(tmp_path))
    guard=ROOT/'common/navigation/mission_guard.sh'
    lock=tmp_path/f'burger1-prepare-{os.getuid()}.lock'
    with lock.open('a') as owner:
        fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for descriptor in ('','999'):
            result=subprocess.run(['bash','-c','source "$1"; mission_guard burger1 "$2"',
                                   'guard',str(guard),descriptor],capture_output=True,text=True)
            assert result.returncode != 0
    result=subprocess.run(['bash','-c','source "$1"; mission_guard burger1',
                           'guard',str(guard)],capture_output=True,text=True)
    assert result.returncode == 0


def test_source_layout_has_no_unused_profile_copies_or_nested_overrides():
    used = {e['source'] for robot in ('M1','M2')
            for e in json.loads((ROOT/robot/'runtime_manifest.json').read_text())['files']}
    assert not (ROOT/'amr').exists()
    assert not (ROOT/'scripts/local_robot.py').exists()
    for robot in ('M1','M2'):
        for directory in ('src','overrides'):
            for path in (ROOT/robot/directory).rglob('*'):
                if path.is_file() and '__pycache__' not in path.parts and path.name != '.gitkeep':
                    assert str(path.relative_to(ROOT)) in used, path
    assert not any('/overrides/' in source for source in used)


@pytest.mark.parametrize('robot', ['M1','M2'])
def test_package_sources_and_camera_dependencies_are_canonical(tmp_path, robot):
    manifest = json.loads((ROOT/robot/'runtime_manifest.json').read_text())
    mapping = {e['target']:e['source'] for e in manifest['files']}
    assert len(mapping)==len(manifest['files'])
    prefix = 'workspace/host_ws/src/waffle_navigation/'
    for target, source in mapping.items():
        if target.startswith(prefix) and not target.startswith((prefix+'config/',prefix+'maps/')):
            assert source=='common/src/waffle_navigation/'+target[len(prefix):]
    for number in ('1','2'):
        assert mapping[prefix+f'config/waypoints_burger{number}.yaml']==f'M{number}/config/waypoints.yaml'
        assert mapping[prefix+f'config/nav2_burger{number}_params.yaml']==f'M{number}/config/nav2.yaml'
        launch = (ROOT/'common/src/waffle_navigation/launch'/f'burger{number}_navigation.launch.py').read_text()
        assert f"'namespace': 'burger{number}'" in launch
        assert "'cmd_vel_out_topic']='cmd_vel_nav_out'" in launch
    for name in ('nav2_waypoints.py','save_start_pose.py'):
        assert os.access(ROOT/'common/src/waffle_navigation/scripts'/name,os.X_OK)
    out = materialize(robot,tmp_path/robot,True)
    camera = Path(json.loads((out/'runtime.json').read_text())['camera'])
    # Resolve local imports without initializing ROS, camera, GPIO or motors.
    names = ['camera_ipc','video_http','data_flow','docking_vision_worker','docking_node','docking_recorder']
    code = 'import importlib.util,sys;sys.path.insert(0,sys.argv[1]);' \
           'assert all(importlib.util.find_spec(n) is not None for n in sys.argv[2:])'
    subprocess.run([sys.executable,'-c',code,str(camera),*names],check=True)
    for path in camera.glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            modules = [a.name for a in node.names] if isinstance(node,ast.Import) else \
                [node.module] if isinstance(node,ast.ImportFrom) and not node.level and node.module else []
            for module in modules:
                name = module.split('.')[0]+'.py'
                if (ROOT/'common/runtime'/name).exists():
                    assert (camera/name).is_file(), (path.name,name)
    assert (out/'final_robot_ws/robot'/manifest['runtime_robot']/'navigation/sequence_runner.py').resolve()==ROOT/'common/navigation/sequence_runner.py'


@pytest.mark.parametrize('selector', ['--robot','--runtime'])
def test_robot_cli_preserves_old_runtime_selection_without_running_robot(tmp_path,monkeypatch,selector):
    import robot as cli
    out = materialize('M2',tmp_path/'.runtime/M2',True)
    config = json.loads((out/'runtime.json').read_text())
    config['repository'] = str(tmp_path)
    (out/'runtime.json').write_text(json.dumps(config))
    monkeypatch.setattr(cli,'ROOT',tmp_path)
    value = 'M2' if selector=='--robot' else str(out)
    monkeypatch.setattr(sys,'argv',['robot.py',selector,value,'status'])
    calls = []
    monkeypatch.setattr(cli.subprocess,'call',lambda command,**kw: calls.append((command,kw)) or 0)
    assert cli.main()==0
    command, options = calls[0]
    assert command[-1]=='status'
    assert command[-2]==str(Path(config['navigation'])/'operation.py')
    assert options['env']['AMR_ROBOT']=='burger2'
