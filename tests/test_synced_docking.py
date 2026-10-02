"""Regression checks for the shared normal-docking policy imported from M1."""
import importlib.util
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


@pytest.mark.parametrize('robot,align,final,turn', [('M1', .0351, .02808, .234), ('M2', .04212, .033696, .2808)])
def test_parking_retains_onboard_profile_limits(robot, align, final, turn):
    c = make_controller(robot, 'parking')
    assert not c.cfg.board_normal_tracking and not c.cfg.inverse_yaw_speed
    assert c.cfg.align_speed_mps == align and c.cfg.final_speed_mps == final
    assert c.cfg.max_angular_rps == turn and c.cfg.angular_speed_scale == 1.
