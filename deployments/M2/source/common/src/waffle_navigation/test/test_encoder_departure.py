"""Encoder distance gates without ROS graph or motor commands."""
from types import SimpleNamespace
import pytest
from test_directional_waypoints import route


def test_distance_and_rollover():
    assert round(.15/route.ENCODER_METRES_PER_TICK) == 2963
    assert route.encoder_delta(-2**31,2**31-1) == 1
    assert route.encoder_delta(2**31-1,-2**31) == -1
    assert route.encoder_backup_counts((103553,144355),(101062,141882)) == (2491,2473)


def backup(monkeypatch, displacements):
    sent=[]
    counts=iter([(100000,100000)]+[(100000-l,100000-r) for l,r in displacements])
    pub=SimpleNamespace(publish=lambda msg:sent.append(msg.twist.linear.x))
    nav=SimpleNamespace(
        _verify_departure_pipeline=lambda:None,
        get_topic_names_and_types=lambda:[],
        wait_until_stopped=lambda **kw:True,
        _check_departure_scan=lambda:None,
        _encoder_settled=lambda:None,
        _encoder_counts=lambda:next(counts),
        create_publisher=lambda *a:pub,
        destroy_publisher=lambda p:None,
        destroy_subscription=lambda s:None,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:route.TwistStamped().header.stamp)),
        frame_prefix='burger1/')
    monkeypatch.setattr(route.rclpy,'ok',lambda:True)
    monkeypatch.setattr(route,'spin_for',lambda *a:None)
    monkeypatch.setattr(route.time,'sleep',lambda *a:None)
    return nav,sent


def test_backup_verifies_both_wheels_and_stops(monkeypatch):
    nav,sent=backup(monkeypatch,[(500,500),(1000,1000),(1500,1500),
                               (2000,2000),(2500,2500),(2920,2920),(2963,2963)])
    route.WaypointNavigator.pre_backup(nav,.15,.045,route.time.monotonic()+20)
    assert -.045 in sent and -.02 in sent
    assert sent[-10:]==[0.]*10


def test_stop_flush_keeps_encoder_feedback_fresh(monkeypatch):
    nav,sent=backup(monkeypatch,[(500,500),(1000,1000),(1500,1500),
                               (2000,2000),(2500,2500),(2920,2920),(2963,2963)])
    clock=[0.]
    received=[0.]
    def sleep(duration):
        clock[0]+=duration
    def spin(node,duration):
        sleep(duration)
        received[0]=clock[0]
    def settled():
        assert clock[0]-received[0] <= .3, 'stop flush starved encoder callbacks'
    nav._encoder_settled=settled
    monkeypatch.setattr(route.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(route.time,'sleep',sleep)
    monkeypatch.setattr(route,'spin_for',spin)
    route.WaypointNavigator.pre_backup(nav,.15,.045,20.)
    assert sent[-10:]==[0.]*10


@pytest.mark.parametrize('values',[
    [(5000,5000)],  # encoder reset/jump
    [(-101,-101)],  # wrong direction
    [(500,0)],     # one wheel stalled
    [(500,500),(1000,1000),(1500,1500),(2000,2000),(2500,2500),(3100,3100)],
    [(500,500),(1000,1000),(1500,1500),(2000,2000),(2500,2500),(2920,2920),(3070,3070)],
])
def test_backup_fault_always_publishes_stop(monkeypatch,values):
    nav,sent=backup(monkeypatch,values)
    with pytest.raises(RuntimeError):
        route.WaypointNavigator.pre_backup(nav,.15,.045,route.time.monotonic()+20)
    assert sent[-10:]==[0.]*10


@pytest.mark.parametrize('age,torque,stamp_ok',[(.31,True,True),(0,False,True),(0,True,False)])
def test_stale_or_unpowered_feedback_rejected(age,torque,stamp_ok):
    nav=SimpleNamespace(encoder_message=SimpleNamespace(
        header=SimpleNamespace(stamp=None),torque=torque,left_encoder=1,right_encoder=1),
        encoder_received_at=route.time.monotonic()-age,_fresh_stamp=lambda *a:stamp_ok)
    with pytest.raises(RuntimeError,match='ENCODER_DEPARTURE_FAILED'):
        route.WaypointNavigator._encoder_counts(nav)


@pytest.mark.parametrize('recovers',[True,False])
def test_settled_waits_for_callbacks_before_validating(monkeypatch,recovers):
    clock=[0.]
    def spin(nav,duration):clock[0]+=duration
    def counts():
        if not recovers or clock[0]<.2:
            raise RuntimeError('ENCODER_DEPARTURE_FAILED: stale')
        return (100,100)
    nav=SimpleNamespace(_encoder_counts=counts)
    monkeypatch.setattr(route.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(route,'spin_for',spin)
    monkeypatch.setattr(route.rclpy,'ok',lambda:True)
    if recovers:
        route.WaypointNavigator._encoder_settled(nav)
        assert clock[0]>=.5
    else:
        with pytest.raises(RuntimeError,match='정지 확인 실패'):
            route.WaypointNavigator._encoder_settled(nav)
        assert clock[0]>=2.
