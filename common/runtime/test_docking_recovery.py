"""Deterministic dropped-frame and recovery tests; no ROS or motor output."""
import pytest
from docking_control import DockingControl, Settings
from test_docking_control import inputs, observation, final_state


def waiting():
    c = DockingControl()
    inputs(c, 0., o=observation(z=.65))
    assert c.start(0.)[0]
    assert c.tick(0.)[0] > 0
    c.set_ir(False, .536)
    c.set_odom(.05, 0, 0, .04, 0, .536)
    assert c.tick(.536) == (0, 0)
    assert c.state == 'VISION_WAIT'
    return c


def recover(c, start=.6):
    for i in range(9):
        t = start+i*.05
        inputs(c, t, x=.05, o=observation(z=.60))
        cmd = c.tick(t)
        if c.state == 'ALIGN':
            assert cmd == (0, 0)
            return t
        assert cmd == (0, 0)
    pytest.fail('did not recover')


def test_measured_536ms_gap_stops_then_recovers_without_resetting_budget():
    c = waiting()
    t = recover(c)
    assert c.reason == 'vision_recovered' and c.vision_recoveries == 1
    assert c.started_at == 0 and c.distance == pytest.approx(.05)
    inputs(c, t+.01, x=.05, o=observation(z=.60))
    assert c.tick(t+.01)[0] > 0


@pytest.mark.parametrize('kind', ['absent', 'invalid', 'moving', 'old_capture'])
def test_no_resume_without_fresh_valid_stationary_input(kind):
    c = waiting()
    for i in range(1, 43):
        t = .536+i*.05
        c.set_ir(False, t)
        c.set_odom(.05, 0, 0, .03 if kind == 'moving' else 0, 0, t)
        if kind != 'absent':
            c.set_observation(observation(z=.6, valid=kind != 'invalid'),
                              t-.3 if kind == 'old_capture' else t)
        assert c.tick(t) == (0, 0)
    assert c.state == 'FAULT' and c.reason == 'vision_recovery_timeout'


def test_one_frame_never_satisfies_recovery_hold():
    c = waiting()
    c.set_observation(observation(z=.6), .6)
    for i in range(20):
        t = .6+i*.02
        c.set_ir(False, t)
        c.set_odom(.05, 0, 0, 0, 0, t)
        assert c.tick(t) == (0, 0)
        assert c.state == 'VISION_WAIT'


@pytest.mark.parametrize('kind, reason', [('ir', 'ir_timeout'), ('odom', 'odometry_timeout'),
                                         ('high', 'ir_high_before_final_approach'),
                                         ('jump', 'odometry_jump')])
def test_other_safety_faults_still_stop_during_wait(kind, reason):
    c = waiting()
    inputs(c, 1., x=.3 if kind == 'jump' else .05, high=kind == 'high')
    if kind == 'ir': c.ir_time = 0
    if kind == 'odom': c.odom_time = 0
    assert c.tick(1.) == (0, 0)
    assert c.state in ('FAULT', 'STOPPED') and c.reason == reason


def test_only_three_recoveries_allowed():
    c = waiting()
    for n in range(1, 4):
        t = recover(c, c.vision_wait_at+.05)
        assert c.vision_recoveries == n
        t += .6
        c.set_ir(False, t)
        c.set_odom(.05, 0, 0, 0, 0, t)
        assert c.tick(t) == (0, 0)
    assert c.state == 'FAULT' and c.reason == 'vision_recovery_limit'


def test_wait_does_not_bypass_total_limit_or_allow_new_start():
    c = waiting()
    assert c.start(.536) == (False, 'already_running')
    c.distance = c.cfg.max_total_distance_m
    assert c.tick(.536) == (0, 0)
    assert c.reason == 'total_approach_limit'


def test_final_approach_never_auto_recovers():
    c = final_state()
    c.set_ir(False, 1.4)
    c.set_odom(0, 0, 0, .03, 0, 1.4)
    assert c.tick(1.4) == (0, 0)
    assert c.state == 'FAULT' and c.reason == 'vision_timeout'


@pytest.mark.parametrize('kwargs', [{'max_vision_recoveries': 1.5},
                                   {'vision_recovery_fresh_s': .6},
                                   {'vision_recovery_hold_s': 2.}])
def test_invalid_recovery_settings_rejected(kwargs):
    with pytest.raises(ValueError): Settings(**kwargs)
