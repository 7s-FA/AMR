from pathlib import Path
import hashlib,importlib.util,json
import pytest,yaml
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('materialize',ROOT/'scripts/materialize.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)

@pytest.mark.parametrize('robot',['M1','M2'])
def test_materialized_runtime_has_all_manifest_files_and_no_host_dependency(tmp_path,robot):
    out=mod.materialize(robot,tmp_path/robot);cfg=json.loads((out/'runtime.json').read_text());manifest=json.loads((ROOT/robot/'runtime_manifest.json').read_text())
    assert manifest['runtime_robot']=='burger'+robot[-1]
    nav=Path(cfg['navigation']);camera=Path(cfg['camera']);ws=Path(cfg['workspace'])
    assert (nav/'action_gate.py').exists() and (nav/'terminal_evidence.py').exists()
    assert '-r __ns:=/'+robot in (out/'start_action.sh').read_text()
    assert json.loads((nav/'action_profile.json').read_text())['action_name']=='/'+robot+'/data'
    assert not (camera/'camera_node.py').exists() and not (camera/'start_nav_base.sh').exists()
    assert not (nav/'run_docking_departure_once.sh').exists()
    assert (nav/'communication_guard.py').read_bytes()==(camera/'communication_guard.py').read_bytes()
    assert (camera/'camera.yaml').exists() and (camera/'docking_vision/board.py').exists()
    assert (ws/'host_ws/src/host_pkg/action/Burger.action').exists()
    assert (ws/'host_ws/src/waffle_navigation/maps/factory_map.pgm').exists()
    assert 'host_ws/setup_host.sh' not in (ws/'handoff/pc_env.bash').read_text()
    assert str(camera) in (ws/'handoff/pc_env.bash').read_text()
    assert 'DEPARTURE_SPEED' in (nav/'run_selected_waypoints.sh').read_text()
    for p in (out/'units').glob('*.service'):
        assert '%h/final_robot_ws' not in p.read_text()
    with pytest.raises(ValueError):mod.materialize(robot,out)

@pytest.mark.parametrize('robot,ids,park',[('M1',[4,5,6,7],['way1_park']),('M2',[8,9,10,11],['way4_park'])])
def test_calibrated_robot_differences_are_preserved(robot,ids,park):
    cfg=yaml.safe_load((ROOT/robot/'config/routes.yaml').read_text());assert [m['id'] for m in yaml.safe_load((ROOT/robot/'config/parking_board.yaml').read_text())['markers']]==ids;assert cfg['routes']['park']==park
    camera=yaml.safe_load((ROOT/robot/'config/camera.yaml').read_text());assert camera

def test_general_docking_uses_one_controller_and_profile_timeouts():
    controls=[]
    for robot in ('M1','M2'):
        m=json.loads((ROOT/robot/'runtime_manifest.json').read_text())
        e=next(e for e in m['files'] if e['target']=='camera/docking_control.py')
        assert e['source']=='common/runtime/docking_control.py'
        cfg=yaml.safe_load((ROOT/robot/'config/docking.yaml').read_text())
        assert cfg['control']['board_normal_tracking'] is True
        assert cfg['parking_control']['board_normal_tracking'] is False
        control=dict(cfg['control'])
        assert control.pop('max_final_time_s') == {'M1':30.,'M2':12.}[robot]
        controls.append(control)
    assert controls[0]==controls[1]

def test_no_generated_or_backup_content_in_manifest():
    for robot in ['M1','M2']:
        for e in json.loads((ROOT/robot/'runtime_manifest.json').read_text())['files']:
            assert not any(x in e['source'] for x in ['.before','.backup','__pycache__','camera_src/'])
