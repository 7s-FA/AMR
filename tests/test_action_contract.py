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
