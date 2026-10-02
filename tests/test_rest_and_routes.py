from pathlib import Path
import json
import sys
import pytest
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'common/navigation'), str(ROOT/'tools/test_host')]
from rest_forward import StopMonitor, settings
from sequence_runner import Sequence
from terminal_evidence import terminal_result, arrival_result
from client import request_values


def test_rest_settings_are_independent():
    cfg = yaml.safe_load((ROOT/'M2/config/docking.yaml').read_text())
    cfg['control']['final_speed_mps'] = 99
    assert settings(cfg)['speed_mps'] == .0351
    cfg['rest_control']['odom_timeout_s'] = 2
    with pytest.raises(ValueError): settings(cfg)


def test_continuous_fresh_stationary_odom_required():
    m = StopMonitor(.3, .5)
    assert not m.stopped(0)
    for t in (0., .2, .4, .6): m.update(t+1, 0, 0, 0, t)
    assert m.stopped(.6)
    assert not m.stopped(1.)
    m.update(2, 0, 0, 0, 1.)
    assert not m.stopped(1.)


@pytest.mark.parametrize('stamp,age,v,w', [(1, 0, 0, 0), (2, 1, 0, 0), (2, 0, .02, 0), (2, 0, 0, .05), (2, 0, float('nan'), 0)])
def test_replayed_stale_moving_invalid_odom_resets_hold(stamp,age,v,w):
    m = StopMonitor(.3,.5); m.update(1,0,0,0,0)
    m.update(stamp,age,v,w,.2)
    assert m.since is None and not m.stopped(.2)


@pytest.mark.parametrize('mode,state', [('dock','DOCKED'), ('park','DOCKED'), ('rest','RESTED')])
def test_arrival_evidence_requires_controller_confirmation(tmp_path,mode,state):
    log=tmp_path/'log'
    log.write_text(json.dumps({'state':state,'reason':'ir_high_and_stationary','stopped':True,'ir_high':True})+'\n')
    evidence=terminal_result(mode,0,log)
    proof=arrival_result(mode,evidence)
    assert proof['terminal_verified'] and proof['dock_verified']==(mode!='rest')
    with pytest.raises(RuntimeError): arrival_result(mode,terminal_result(mode,1,log))
    log.write_text('process exited normally\n')
    with pytest.raises(RuntimeError): arrival_result(mode,terminal_result(mode,0,log))


def test_routes_use_yaml_and_reject_invalid_before_movement():
    calls=[]; stations={'routes':{'asm':['way3']},'terminal':{'asm':'rest'}}
    seq=Sequence(calls.append,calls.append,stations)
    assert seq.execute('asm')=='rest' and calls[-1]=='rest'
    calls.clear();stations['terminal']['asm']='unknown'
    with pytest.raises(ValueError):seq.execute('asm')
    assert calls==[]


def test_failed_waypoint_never_starts_terminal():
    calls=[]
    def fail(_):raise RuntimeError('navigation failed')
    with pytest.raises(RuntimeError):Sequence(fail,calls.append,{'routes':{'asm':[]},'terminal':{'asm':'dock'}}).execute('asm')
    assert calls==[]


@pytest.mark.parametrize('robot',['M1','M2'])
def test_test_host_uses_exact_uppercase_contract(robot):
    assert request_values(robot,'asm',50)==('/'+robot+'/data','GO_TO_ASM',50)
    assert request_values(robot,'stop',100)[1:]==('EMER_STOP',0)
    with pytest.raises(ValueError):request_values(robot,'asm',0)
    with pytest.raises(ValueError):request_values(robot.lower(),'asm',100)


@pytest.mark.parametrize('verified',[True,False])
def test_wrapper_only_sets_departure_flag_with_verified_stop(tmp_path,verified):
    import shutil
    import subprocess
    nav=tmp_path/'robot/burger2/navigation';nav.mkdir(parents=True)
    for name in ('terminal_wrapper.sh','terminal_evidence.py'):shutil.copy2(ROOT/'common/navigation'/name,nav/name)
    (nav/'set_mode.sh').write_text('exit 0\n')
    controller=tmp_path/'controller.sh'
    controller.write_text("echo '"+json.dumps({'state':'RESTED','reason':'ir_high_and_stationary','stopped':verified,'ir_high':True})+"'\n")
    result=subprocess.run(['bash',str(nav/'terminal_wrapper.sh'),'rest',str(controller)],capture_output=True,text=True)
    assert (result.returncode==0)==verified
    assert (tmp_path/'data/burger2/departure_pending').exists()==verified


def test_odom_age_includes_transport_delay():
    m=StopMonitor(.3,.5)
    m.update(1,.25,0,0,10.)
    assert m.fresh(10.04)
    assert not m.fresh(10.06)
