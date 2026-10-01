"""Regression for the near-stage oscillation fix ported from Burger2."""
import math
from docking_control import DockingControl, Settings
from test_docking_control import inputs, observation


def test_small_near_stage_heading_error_does_not_saturate_turn():
    c = DockingControl(Settings(lateral_tolerance_m=.005, yaw_tolerance_deg=2,
                                max_angular_rps=.18, align_speed_mps=.03))
    inputs(c, 0., o=observation(x=-.015, z=.25, yaw=-.04))
    assert c.start(0.)[0]
    v, w = c.tick(0.)
    assert c.reason == 'near_stage_heading_alignment'
    assert v == 0.
    assert math.isclose(w, .048)


def test_staging_point_behind_robot_uses_forward_correction():
    c = DockingControl(Settings(lateral_tolerance_m=.005, yaw_tolerance_deg=2,
                                max_angular_rps=.18, align_speed_mps=.03))
    inputs(c, 0., o=observation(x=.01, z=.248))
    assert c.start(0.)[0]
    v, w = c.tick(0.)
    assert c.reason == 'near_stage_forward_alignment'
    assert 0. < v <= .03
    assert -.18 <= w < 0.
