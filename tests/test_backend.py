from pathlib import Path
import sys,json,subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'common/navigation'),str(ROOT/'common/src/amr_mission')]
from amr_mission.backend import Backend

@pytest.fixture
def backend(tmp_path):
    nav=tmp_path/'robot/burger2/navigation';nav.mkdir(parents=True)
    b=Backend(nav)
    def run(args,**kwargs):
        if args[0]=='pgrep':return subprocess.CompletedProcess(args,1,'','')
        if args[:3]==['systemctl','--user','show']:return subprocess.CompletedProcess(args,0,'inactive\n','')
        return subprocess.CompletedProcess(args,0,'','')
    b.op.run=run
    b.op.data.mkdir(parents=True,exist_ok=True)
    b.atomic(b.op.data/'operation_mode.json',{'mode':'process'})
    return b

def test_emergency_stop_blocks_legacy_cli_until_restart(backend):
    backend.stop()
    with pytest.raises(RuntimeError):backend.op.submit('asm',source='host',task_id='blocked')
    backend.restart();assert not backend.load_gate(backend.path)['estop']
    assert backend.op.read('mission_state.json') is None

def test_goal_uuid_is_mission_id_and_renewal_never_clears_estop(backend):
    task=backend.submit('GO_TO_ASM',50.,'goal-001');assert task['command_id']=='goal-001'
    assert backend.renew('goal-001')
    backend.stop();assert not backend.renew('goal-001')
    backend.finish('goal-001',True);assert backend.load_gate(backend.path)['estop']

def test_server_restart_latches_unfinished_goal(backend):
    backend.submit('GO_TO_PARK',100.,'old-goal')
    b=Backend(backend.nav);assert b.load_gate(b.path)['estop']

def test_individual_mode_rejects_host_submit(backend):
    backend.atomic(backend.op.data/'operation_mode.json',{'mode':'individual'})
    with pytest.raises(RuntimeError):backend.submit('GO_TO_REST',100.,'wrong-mode')
