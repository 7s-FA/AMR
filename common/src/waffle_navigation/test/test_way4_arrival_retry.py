"""Regression: AMCL moves a successful position goal outside 2cm on stopping."""
import pytest
from test_position_then_yaw import navigator, POINTS
from test_directional_waypoints import ROOT, route


def test_single_pre_yaw_correction_reaches_original_goal(monkeypatch):
    nav = navigator(monkeypatch)
    nav.position_arrival_retries = 1
    calls=[]
    def finish(*args, **kw):
        calls.append(kw)
        nav.position_retry_before_yaw = len(calls) == 1
        return len(calls) > 1
    monkeypatch.setattr(route, 'finish_waypoint', finish)
    assert route.run_waypoints(nav, POINTS[:1], 60, ROOT/'behavior_trees') == 0
    assert len(nav.goals) == 2
    assert nav.goals[0][0].pose.position.x == nav.goals[1][0].pose.position.x
    assert nav.goals[0][0].pose.position.y == nav.goals[1][0].pose.position.y


@pytest.mark.parametrize('pre_yaw', [False, True])
def test_retry_is_bounded_and_never_allowed_after_yaw(monkeypatch, pre_yaw):
    nav = navigator(monkeypatch)
    nav.position_arrival_retries = 1
    def fail(*args, **kw):
        nav.position_retry_before_yaw = pre_yaw
        return False
    monkeypatch.setattr(route, 'finish_waypoint', fail)
    assert route.run_waypoints(nav, POINTS[:1], 60, ROOT/'behavior_trees') == 1
    assert len(nav.goals) == (2 if pre_yaw else 1)


@pytest.mark.parametrize('distance, eligible', [(0.048, True), (0.081, False)])
def test_only_small_position_errors_before_spin_are_eligible(monkeypatch, distance, eligible):
    nav = navigator(monkeypatch)
    nav.xy_tolerance = 0.02
    nav.current_map_pose=lambda timeout=2: (POINTS[0]['x']+distance, POINTS[0]['y'], 0.)
    assert not route.finish_waypoint(nav, POINTS[0], route.time.monotonic()+10, 1, is_final=True)
    assert nav.position_retry_before_yaw is eligible
    assert not nav.spin_requests
