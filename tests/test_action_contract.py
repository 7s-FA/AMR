import importlib.util,json,math,sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'common/navigation'),str(ROOT/'common/src/amr_mission')]
from action_gate import gate_output,departure_speed
from amr_mission.backend import validate,ROUTES

@pytest.mark.parametrize('command',list(ROUTES)+['EMER_STOP','RESTART'])
def test_commands(command):validate(command,100.)
@pytest.mark.parametrize('command,speed',[('move_to_warehouse',100),('GO_TO_MAT',0),('GO_TO_ASM',-1),('GO_TO_ASM',101),('GO_TO_ASM',float('nan')),('GO_TO_ASM',float('inf'))])
def test_bad_commands(command,speed):
    with pytest.raises(ValueError):validate(command,speed)
def test_stop_accepts_zero():validate('EMER_STOP',0.)

def write(tmp_path,**updates):
    d={'estop':False,'goal_id':'abc','heartbeat':10.,'speed_percent':50.,'max_linear_mps':.066,'max_angular_rps':.924};d.update(updates)
    p=tmp_path/'gate.json';p.write_text(json.dumps(d));return p

def test_speed_is_percent_ceiling_not_velocity(tmp_path):
    p=write(tmp_path)
    assert gate_output(p,.066,-.924,10.1)==pytest.approx((.033,-.462))
    assert gate_output(p,.01,.1,10.1)==pytest.approx((.01,.1))
@pytest.mark.parametrize('change',[{'estop':True},{'heartbeat':8.},{'heartbeat':11.},{'speed_percent':-1},{'speed_percent':101},{'max_linear_mps':10}])
def test_latched_stale_invalid_gate_stops(tmp_path,change):assert gate_output(write(tmp_path,**change),.06,.2,10.1)==(0.,0.)
def test_broken_gate_stops(tmp_path):
    p=tmp_path/'gate.json';p.write_text('{');assert gate_output(p,.06,.2,10.)==(0.,0.)
def test_restart_does_not_restore_motion(tmp_path):
    p=write(tmp_path,goal_id=None)
    assert gate_output(p,0,0,10.)==(0,0)
def test_missing_gate_preserves_individual_mode(tmp_path):assert gate_output(tmp_path/'missing',.02,.1,10.)==(.02,.1)

@pytest.mark.parametrize('percent,expected',[(100,.0858),(50,.0429)])
def test_thirty_percent_upgrade_keeps_percentage_ceiling(tmp_path,percent,expected):
    p=write(tmp_path,max_linear_mps=.0858,speed_percent=percent)
    assert gate_output(p,.2,-.2,10.1)==pytest.approx((expected,-.2))
    assert gate_output(p,-.2,.2,10.1)==pytest.approx((-expected,.2))

def test_upgraded_ceiling_does_not_accelerate_docking_or_departure(tmp_path):
    p=write(tmp_path,max_linear_mps=.0858,speed_percent=100,heartbeat=__import__('time').monotonic())
    assert departure_speed(p)==pytest.approx(.045)
    assert gate_output(p,.035,.1)==pytest.approx((.035,.1))

def test_above_requested_ceiling_still_stops(tmp_path):
    assert gate_output(write(tmp_path,max_linear_mps=.151,speed_percent=100),.1,.1,10.1)==(0.,0.)

@pytest.mark.parametrize('percent,expected',[(100,.15),(50,.075)])
def test_fifteen_cm_limit_and_departure_preserved(tmp_path,percent,expected):
    p=write(tmp_path,max_linear_mps=.15,speed_percent=percent)
    assert gate_output(p,.3,.1,10.1)==pytest.approx((expected,.1))
    p=write(tmp_path,max_linear_mps=.15,speed_percent=100,heartbeat=__import__('time').monotonic())
    assert departure_speed(p)==pytest.approx(.045)
    assert gate_output(p,.035,.1)==pytest.approx((.035,.1))
