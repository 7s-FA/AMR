"""Behavior tests including differential-drive alignment and latched IR stops."""
import math
import pytest

from docking_control import DockingControl, Settings, wrap


def observation(x=0., z=.25, yaw=0., valid=True):
    return {'pose_valid': valid, 'tvec_m': [x, 0., z], 'normal_yaw_rad': yaw,
            'reason': 'alignment_needed' if valid else 'markers_missing'}


def inputs(c, t, *, high=False, o=None, x=0., y=0., yaw=0., v=0., w=0.):
    c.set_ir(high, t)
    c.set_odom(x, y, yaw, v, w, t)
    c.set_observation(o if o is not None else observation(), t)


def final_state():
    c = DockingControl()
    inputs(c, 0)
    assert c.start(0)[0]
    for i in range(81):
        t = i*.01
        inputs(c, t)
        c.tick(t)
    assert c.state == 'FINAL_APPROACH'
    return c


def test_ir_high_stops_with_missing_markers_and_stays_latched():
    c = final_state()
    inputs(c, .9, o=observation(valid=False))
    assert c.tick(.9) == (.03, 0.)
    inputs(c, 1., high=True, o=observation(valid=False), v=.03)
    assert c.last_command == (0., 0.)  # GPIO callback itself clears command.
    assert c.tick(1.) == (0., 0.) and c.state == 'STOPPING'
    for i in range(101, 165):
        inputs(c, i*.01, o=observation(valid=False))  # HIGH need not remain asserted.
        assert c.tick(i*.01) == (0., 0.)
    assert c.state == 'DOCKED'
    inputs(c, 1.7)
    assert c.tick(1.7) == (0., 0.)


def test_start_requires_fresh_low_ir_stationary_odom_and_four_markers():
    c = DockingControl()
    assert not c.start(0)[0]
    inputs(c, 1, high=True)
    assert c.start(1) == (False, 'ir_already_high')
    inputs(c, 2, o=observation(valid=False))
    assert c.start(2) == (False, 'four_markers_required')
    inputs(c, 3, v=.03)
    assert c.start(3) == (False, 'robot_must_be_stationary')
    inputs(c, 4, o=observation(z=2))
    assert c.start(4) == (False, 'start_distance_out_of_range')


def test_loss_before_alignment_never_enters_blind_approach():
    c = DockingControl()
    inputs(c, 0)
    assert c.start(0)[0]
    for i in range(100):
        inputs(c, i*.02, o=observation(valid=False))
        assert c.tick(i*.02) == (0., 0.)
    assert c.state == 'ALIGN'


def test_parking_30cm_alignment_then_straight_until_ir():
    c = DockingControl(Settings(staging_distance_m=.30,
                               lateral_tolerance_m=.001, yaw_tolerance_deg=1.))
    inputs(c, 0., o=observation(z=.31))
    assert c.start(0.)[0]
    # A visible board with incorrect alignment must not latch straight motion.
    for i in range(1, 91):
        inputs(c, i*.01, o=observation(x=.01, z=.31, yaw=.1))
        c.tick(i*.01)
        assert c.state == 'ALIGN'
    for i in range(91, 181):
        inputs(c, i*.01, o=observation(z=.31))
        c.tick(i*.01)
    assert c.state == 'FINAL_APPROACH'
    invalid = observation(z=.27, valid=False)
    invalid['reason'] = 'board_geometry_inconsistent'
    inputs(c, 1.9, o=invalid)
    assert c.tick(1.9) == (.03, 0.)
    inputs(c, 2., high=True, o=invalid)
    assert c.tick(2.) == (0., 0.) and c.state == 'STOPPING'
    for i in range(201, 261):
        inputs(c, i*.01, o=invalid)
        assert c.tick(i*.01) == (0., 0.)
    assert c.state == 'DOCKED'


def test_alignment_hold_must_be_continuous_and_stationary():
    c = DockingControl()
    inputs(c, 0)
    c.start(0)
    for i in range(65):
        inputs(c, i*.01)
        c.tick(i*.01)
    c.set_observation(observation(valid=False), .65)
    c.set_observation(observation(), .66)  # Even a brief loss between ticks breaks hold.
    c.tick(.66)
    assert c.state == 'ALIGN'
    for i in range(67, 130):
        inputs(c, i*.01, v=.03)
        c.tick(i*.01)
    assert c.state == 'ALIGN' and c.aligned_since is None


@pytest.mark.parametrize('kind', ['ir', 'odom', 'vision'])
def test_stream_failure_stops_even_during_final_approach(kind):
    c = final_state()
    inputs(c, .9, o=observation(valid=False))
    setattr(c, {'ir': 'ir_time', 'odom': 'odom_time', 'vision': 'observation_time'}[kind], 0.)
    assert c.tick(.9) == (0., 0.)
    assert c.state == 'FAULT'


def test_final_travel_and_time_limits_stop_when_ir_never_arrives():
    c = final_state()
    for i in range(10):
        inputs(c, 1+i*.05, x=.04*(i+1))
        c.tick(1+i*.05)
    assert c.state == 'FAULT' and c.reason == 'final_approach_limit'
    c = final_state()
    inputs(c, 14.)
    assert c.tick(14.) == (0., 0.) and c.reason == 'final_approach_limit'


def test_ir_before_final_does_not_claim_docking_success():
    c = DockingControl()
    inputs(c, 0, o=observation(z=.5))
    c.start(0)
    inputs(c, .1, high=True)
    assert c.tick(.1) == (0., 0.) and c.state == 'STOPPED'
    inputs(c, .2)
    assert c.tick(.2) == (0., 0.)


def test_odometry_jump_stops_final_approach():
    c = final_state()
    inputs(c, .9, x=2)
    assert c.tick(.9) == (0., 0.) and c.reason == 'odometry_jump'


@pytest.mark.parametrize('reason', ['markers_missing', 'duplicate_target_id', 'pose_failed',
                                  'board_geometry_inconsistent'])
def test_final_approach_ignores_marker_quality_until_ir_high(reason):
    c = final_state()
    o = observation(valid=False)
    o['reason'] = reason
    inputs(c, .9, o=o)
    assert c.tick(.9) == (.03, 0.) and c.state == 'FINAL_APPROACH'
    inputs(c, 1., high=True, o=o)
    assert c.tick(1.) == (0., 0.) and c.state == 'STOPPING'


@pytest.mark.parametrize('offset,heading', [(0., 0.), (.06, 0.), (-.06, .1), (.03, -.15), (0., .2)])
def test_differential_drive_reaches_alignment_with_correct_turn_sign(offset, heading):
    c = DockingControl()
    x, y, a, v, w = 0., offset, heading, 0., 0.
    for i in range(4500):
        t = i*.02
        dx, dy = .55-x, -y
        bx, by = math.cos(a)*dx+math.sin(a)*dy, -math.sin(a)*dx+math.cos(a)*dy
        inputs(c, t, x=x, y=y, yaw=a, v=v, w=w, o=observation(-by, bx, a))
        if i == 0:
            assert c.start(t)[0]
        v, w = c.tick(t)
        assert 0 <= v <= .04 and abs(w) <= .25
        if c.state == 'FINAL_APPROACH':
            assert abs(by) <= .005 and abs(a) <= math.radians(2)
            return
        assert c.state != 'FAULT', c.reason
        # Mirror OpenCR command quantization.
        v, w = int(v*100)/100, int(w*100)/100
        x += v*math.cos(a)*.02
        y += v*math.sin(a)*.02
        a = wrap(a+w*.02)
    pytest.fail('Did not reach final approach')


def test_settings_reject_unsafe_numeric_values():
    for params in [{'final_speed_mps': float('nan')}, {'aligned_hold_s': 0},
                   {'final_speed_mps': .2}, {'max_angular_rps': 1.}]:
        with pytest.raises(ValueError):
            Settings(**params)
