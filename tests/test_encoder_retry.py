from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'common/navigation'))
from sequence_runner import dock_retry_backup, run_step, docking_retry_needs_backup
from mission_retry import RetryPolicy


def test_docking_retry_backs_up_without_turn_or_route(tmp_path):
    calls=[]
    dock_retry_backup(tmp_path,tmp_path,{},run=lambda args,**kw:calls.append(args))
    assert calls[0][-1]=='nav' and calls[-1][-1]=='idle'
    command=calls[1]
    assert '--encoder-backup-only' in command
    assert '--pre-backup-open-loop' in command
    distance=float(command[command.index('--pre-backup-distance')+1])
    speed=float(command[command.index('--pre-backup-speed')+1])
    assert distance/speed==pytest.approx(3.5)
    assert command[command.index('--pre-turn-angle-deg')+1]=='0'


def test_failed_recovery_never_repeats_reverse(tmp_path):
    calls=[]; operations=[]; stops=[]
    def run(args,**kw):
        calls.append(args)
        if '--encoder-backup-only' in args:raise RuntimeError('stale encoder')
    def operation():
        operations.append(1)
        raise RuntimeError('dock failed')
    policy=RetryPolicy(3,lambda:stops.append(1),
        lambda stage:dock_retry_backup(tmp_path,tmp_path,{},run),lambda *a:None)
    with pytest.raises(RuntimeError,match='자동 재시도 금지'):
        policy.execute('terminal_dock',operation)
    assert len(operations)==1
    assert sum('--encoder-backup-only' in c for c in calls)==1
    assert calls[-1][-1]=='idle'


def test_docking_retries_after_successful_reverse():
    events=[]
    def operation():
        events.append('dock')
        if events.count('dock')==1:raise RuntimeError('dock failed')
    policy=RetryPolicy(3,lambda:events.append('stop'),
                       lambda s:events.append('reverse'),lambda *a:None)
    policy.execute('terminal_dock',operation)
    assert events==['dock','stop','reverse','dock']


def test_departure_failure_marker_survives_action_error_tail():
    with pytest.raises(RuntimeError,match='ENCODER_DEPARTURE_FAILED'):
        run_step(['/bin/bash','-c',
                  'echo "Nav2 error_code=700"; echo "ENCODER_DEPARTURE_FAILED: turn failed"; exit 1'])


@pytest.mark.parametrize('delta,reverse',[
    ((0,0),False),((2962,2962),False),((2963,2963),True),
    ((2962,2964),True),((4000,4000),True),((-4965,4965),False)])
def test_retry_distance_boundary_and_rotation(delta,reverse):
    evidence={'status':'failed','controller_report':{'docking_encoder':{
        'valid':True,'start':[100,200],'current':[100+delta[0],200+delta[1]]}}}
    assert docking_retry_needs_backup(evidence) is reverse


@pytest.mark.parametrize('sample',[
    None,{}, {'valid':False,'start':[0,0],'current':[4000,4000]},
    {'valid':True,'start':None,'current':[4000,4000]},
    {'valid':True,'start':[0,0],'current':None},
    {'valid':True,'start':[0,0],'current':[True,4000]},
])
def test_missing_or_stale_encoder_never_falls_back(sample):
    with pytest.raises(RuntimeError,match='ENCODER_DEPARTURE_FAILED'):
        docking_retry_needs_backup({'status':'failed','controller_report':{'docking_encoder':sample}})


def test_encoder_evidence_survives_terminal_log(tmp_path):
    import json
    from terminal_evidence import terminal_result
    log=tmp_path/'terminal.log'
    sample={'valid':True,'start':[0,0],'current':[2963,2963]}
    log.write_text(json.dumps({'state':'FAULT','reason':'vision_recovery_timeout','docking_encoder':sample})+'\n')
    assert docking_retry_needs_backup(terminal_result('dock',1,log))
