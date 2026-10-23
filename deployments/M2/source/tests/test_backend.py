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

def test_server_restart_revokes_unfinished_goal_without_latch(backend):
    backend.submit('GO_TO_PARK',100.,'old-goal')
    b=Backend(backend.nav);gate=b.load_gate(b.path)
    assert not gate['estop'] and gate['goal_id'] is None and not b.authorized('old-goal')

def test_only_emergency_stop_latches(backend):
    # 취소·실패·종료는 이동만 멈추고 정지 래치를 걸지 않는다.
    for reason in ('action_cancelled','action_failed','action_server_shutdown'):
        backend.submit('GO_TO_ASM',50.,'goal-'+reason)
        backend.stop(reason);gate=backend.load_gate(backend.path)
        assert not gate['estop'] and gate['goal_id'] is None
        backend.finish('goal-'+reason,False);assert not backend.load_gate(backend.path)['estop']
    # EMER_STOP 은 래치를 걸고, 이후 실패 정리로도 풀리지 않는다. RESTART 로만 풀린다.
    backend.stop();backend.stop('action_failed');backend.finish('x',False)
    assert backend.load_gate(backend.path)['estop']
    with pytest.raises(RuntimeError,match='ESTOP_LATCHED'):backend.preflight()
    backend.restart();assert not backend.load_gate(backend.path)['estop']

def test_individual_mode_rejects_host_submit(backend):
    backend.atomic(backend.op.data/'operation_mode.json',{'mode':'individual'})
    with pytest.raises(RuntimeError):backend.submit('GO_TO_REST',100.,'wrong-mode')


def test_authorization_read_does_not_renew_or_clear_latch(backend):
    backend.submit('GO_TO_ASM',50.,'goal-read')
    before=backend.path.read_bytes()
    assert backend.authorized('goal-read')
    assert not backend.authorized('different-goal')
    assert backend.path.read_bytes()==before
    backend.stop()
    assert not backend.authorized('goal-read')


def test_reset_parked_runs_parked_preparation_without_touching_latch(backend,monkeypatch):
    import amr_mission.backend as module
    calls=[]
    def run(command,**kwargs):
        calls.append(command);return subprocess.CompletedProcess(command,0,'ready','')
    monkeypatch.setattr(module.subprocess,'run',run)
    backend.stop()  # 긴급정지 래치는 주차 복구로 풀리지 않는다.
    assert backend.reset_parked('goal-park')
    assert calls[-1][-3:]==[str(backend.nav),backend.op.robot,'parked-goal-park']
    assert '--parked' in calls[-1][2] and backend.load_gate(backend.path)['estop']
    def fail(command,**kwargs):return subprocess.CompletedProcess(command,1,'','RuntimeError: 이동 중')
    monkeypatch.setattr(module.subprocess,'run',fail)
    with pytest.raises(RuntimeError,match='PARKED_RESET_FAILED'):backend.reset_parked('goal-2')
