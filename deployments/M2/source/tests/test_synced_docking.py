"""Regression checks for the shared normal-docking policy imported from M1."""
import importlib.util
from dataclasses import asdict, replace
import math
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('synced_docking_control', ROOT/'common/runtime/docking_control.py')
control = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = control
spec.loader.exec_module(control)


def make_controller(robot, mode='normal'):
    cfg = yaml.safe_load((ROOT/robot/'config/docking.yaml').read_text())
    values = dict(cfg['control'])
    if mode == 'parking':
        values.update(cfg['parking_control'])
    return control.DockingControl(control.Settings(**values))


def observe(c, now, lateral=.02, yaw=5., distance=.5):
    c.set_ir(False, now)
    c.set_odom(0., 0., 0., 0., 0., now)
    c.set_observation({'pose_valid': True, 'tvec_m': [lateral, 0., distance],
                       'normal_yaw_rad': math.radians(yaw)}, now)


@pytest.mark.parametrize('lateral,yaw', [(0., 0.), (.02, 5.), (-.02, -5.), (.03, 20.), (-.03, -20.)])
def test_both_profiles_produce_identical_normal_docking_commands(lateral, yaw):
    a, b = make_controller('M1'), make_controller('M2')
    for c in (a, b):
        observe(c, 0., lateral, yaw)
        c.start(0.)
    for tick in range(1, 31):
        now = tick*.02
        for c in (a, b):
            observe(c, now, lateral, yaw)
        assert a.tick(now) == b.tick(now)
        assert (a.state, a.reason) == (b.state, b.reason)
        assert abs(a.last_command[1]) <= a.cfg.max_angular_rps*a.cfg.angular_speed_scale


@pytest.mark.parametrize('robot', ['M1', 'M2'])
def test_shared_controller_stops_immediately_on_stale_vision_or_ir(robot):
    c = make_controller(robot)
    observe(c, 0.)
    c.start(0.)
    observe(c, .02)
    c.tick(.02)
    c.set_ir(False, .6)
    c.set_odom(0., 0., 0., 0., 0., .6)
    assert c.tick(.6) == (0., 0.)
    assert c.state == 'VISION_WAIT'
    c.set_ir(True, .61)
    assert c.tick(.61) == (0., 0.) and c.state == 'STOPPED'


@pytest.mark.parametrize('robot', ['M1', 'M2'])
def test_parking_retains_snapshot_profile_limits(robot):
    c = make_controller(robot, 'parking')
    assert not hasattr(c.cfg, 'board_normal_tracking')
    assert not hasattr(c.cfg, 'inverse_yaw_speed')
    assert c.cfg.align_speed_mps == .0351 and c.cfg.final_speed_mps == .02808
    assert c.cfg.max_angular_rps == .234 and c.cfg.angular_speed_scale == 1.


@pytest.mark.parametrize('robot', ['M1', 'M2'])
def test_normal_matches_parking_except_staging_distance(robot):
    normal = asdict(make_controller(robot).cfg)
    parking = asdict(make_controller(robot, 'parking').cfg)
    assert normal.pop('staging_distance_m') == .25
    assert parking.pop('staging_distance_m') == .30
    assert normal == parking


@pytest.mark.parametrize('robot', ['M1', 'M2'])
@pytest.mark.parametrize('lateral,yaw,distance', [(0., 0., .26), (.02, 5., .5),
                                               (-.02, -5., .5), (.03, 20., .3),
                                               (.007, 4., .163)])
def test_normal_commands_match_parking_with_25cm_target(robot,lateral,yaw,distance):
    normal = make_controller(robot)
    parking = make_controller(robot, 'parking')
    parking.cfg = replace(parking.cfg, staging_distance_m=.25)
    for c in (normal, parking):
        observe(c, 0., lateral, yaw, distance)
        assert c.start(0.)[0]
    for tick in range(1, 51):
        now = tick*.02
        for c in (normal, parking):
            observe(c, now, lateral, yaw, distance)
        assert normal.tick(now) == parking.tick(now)
        assert (normal.state, normal.reason) == (parking.state, parking.reason)


@pytest.mark.parametrize('robot', ['M1', 'M2'])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_pd_damps_rotation_toward_target(robot,sign):
    c = make_controller(robot)
    assert c.cfg.angular_kd == .3
    c.set_odom(0.,0.,0.,0.,sign*.1,0.)
    assert c.turn(sign*.12,settling=True) == pytest.approx(sign*.09)
    c.set_odom(0.,0.,0.,0.,-sign*.1,1.)
    assert c.turn(sign*.12,settling=True) == pytest.approx(sign*.15)


def test_pd_filters_samples_and_resets_after_odom_gap():
    c = make_controller('M1')
    c.set_odom(0.,0.,0.,0.,0.,0.)
    c.set_odom(0.,0.,0.,0.,.2,.1)
    assert c.filtered_angular == pytest.approx(.1)
    c.set_odom(0.,0.,0.,0.,-.2,1.)
    assert c.filtered_angular == pytest.approx(-.2)


@pytest.mark.parametrize('robot', ['M1', 'M2'])
def test_pd_never_overrides_stale_odom_stop_or_final_straight(robot):
    c = make_controller(robot)
    observe(c,0.);assert c.start(0.)[0]
    c.set_odom(0.,0.,0.,0.,.2,.01)
    c.state='FINAL_APPROACH';c.final_at=.01
    assert c.tick(.02)==(c.cfg.final_speed_mps,0.)
    c.set_ir(False,.4)
    c.set_observation(c.observation,.4)
    assert c.tick(.4)==(0.,0.)
    assert c.state=='ODOM_WAIT'


@pytest.mark.parametrize('value', [-1.,float('nan'),float('inf'),True])
def test_invalid_pd_gain_rejected(value):
    with pytest.raises(ValueError):
        control.Settings(angular_kd=value)


@pytest.mark.parametrize('robot', ['M1', 'M2'])
def test_pd_integrated_alignment_and_ir_stop(robot):
    pd = make_controller(robot)
    p = make_controller(robot)
    p.cfg = replace(p.cfg, angular_kd=0.)
    for c in (pd,p):
        # Board-normal pose near staging selects stationary heading alignment.
        theta = math.radians(5.)
        observe(c,0.,lateral=-.25*math.tan(theta),yaw=-5.,distance=.25)
        assert c.start(0.)[0]
        c.set_odom(0.,0.,0.,0.,.1,.01)
    _, p_rate = p.tick(.02)
    _, pd_rate = pd.tick(.02)
    assert pd.reason == p.reason == 'near_stage_heading_alignment'
    assert 0 < pd_rate < p_rate
    pd.set_ir(True,.03)
    assert pd.tick(.03) == (0.,0.)
    assert pd.state == 'STOPPED'
