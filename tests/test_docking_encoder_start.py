"""Execute the node's encoder methods without constructing ROS/GPIO hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace
import pytest
from collections import deque

SOURCE=Path(__file__).resolve().parents[1]/'common/runtime/docking_node.py'
tree=ast.parse(SOURCE.read_text())
node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DockingNode')
names={'current_encoder','encoder_retry_report','prepare_start','_motor_state','encoder_error'}
methods=[n for n in node.body if isinstance(n,ast.FunctionDef) and n.name in names]
clock=[10.]
scope={'time':SimpleNamespace(monotonic=lambda:clock[0])}
exec(compile(ast.Module(body=methods,type_ignores=[]),str(SOURCE),'exec'),scope)


def fixture():
    clock[0]=10.
    n=SimpleNamespace(encoder_message=None,encoder_received_at=None,docking_encoder_start=None,
                      encoder_samples=deque(maxlen=10),encoder_start_samples=[],encoder_attempt_started=None,
                      stamp_fresh=lambda stamp,limit:stamp=='fresh',execute=True,
                      motor_error=lambda now:None,
                      control=SimpleNamespace(start_error=lambda now:None,start=lambda now:(True,'ALIGN')))
    for name in names:setattr(n,name,scope[name].__get__(n))
    return n


def sample(left,right,stamp='fresh'):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp),torque=True,left_encoder=left,right_encoder=right)


def test_start_pair_remains_fixed_and_new_attempt_resets_it():
    n=fixture();n._motor_state(sample(100,200))
    assert n.prepare_start(10)[0]
    assert n.docking_encoder_start is None
    n._motor_state(sample(100,200))
    for _ in range(3):n._motor_state(sample(3063,3163))
    assert n.encoder_retry_report()=={'valid':True,'start':(100,200),'current':(3063,3163)}
    assert n.prepare_start(10)[0]
    assert n.docking_encoder_start is None
    assert len(n.encoder_samples)==0
    assert n.encoder_start_samples==[]


def test_start_does_not_wait_for_encoder_and_grace_is_five_seconds():
    n=fixture()
    assert n.prepare_start(10)[0]
    assert n.encoder_error(14.999) is None
    assert n.encoder_error(15)=='encoder_samples_insufficient'
    assert n.encoder_retry_report()['valid'] is False


def test_missing_current_after_start_is_not_replaced_by_start_pair():
    n=fixture();n.prepare_start(10);n._motor_state(sample(100,200))
    clock[0]=15.01
    assert n.encoder_retry_report()=={'valid':False,'start':(100,200),'current':None}


@pytest.mark.parametrize('count,failed',[(0,True),(3,True),(4,False),(10,False)])
def test_rolling_window_threshold(count,failed):
    n=fixture();n.prepare_start(10)
    clock[0]=14.
    for _ in range(count):n._motor_state(sample(100,200))
    clock[0]=15.
    assert (n.encoder_error(15) is not None)==failed
    clock[0]=19.01
    assert n.encoder_error(clock[0])=='encoder_samples_insufficient'


def test_ten_sample_bound_and_no_point_three_second_encoder_expiry():
    n=fixture();n.prepare_start(10)
    for i in range(20):n._motor_state(sample(i,i,stamp='stale'))
    clock[0]=11.
    assert len(n.encoder_samples)==10
    assert n.encoder_start_samples==[(i,i) for i in range(10)]
    assert n.docking_encoder_start==(0,0)
    assert n.current_encoder()==(19,19)
    assert n.encoder_retry_report()['start']==n.encoder_start_samples[0]
    assert n.prepare_start(11)[0]
    for i in range(100,110):n._motor_state(sample(i,i))
    assert n.encoder_start_samples==[(i,i) for i in range(100,110)]
    assert n.encoder_retry_report()['start']==(100,100)
