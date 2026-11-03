"""No motion: route sequencing and independent stopped-pose acceptance."""
import math
import pytest
from test_directional_waypoints import ROOT, route, FakeNavigator


@pytest.mark.parametrize('xy,yaw,success', [(0.01, 2, True), (.021, 0, False), (.01, 3.1, False)])
def test_precision_arrival_is_strict_for_intermediate_and_final(monkeypatch, xy, yaw, success):
    nav = FakeNavigator()
    nav.nav2_precision_pose = True
    nav.verify_precision_controller = lambda: None
    points = [dict(x=0., y=0., yaw=0., mode='forward')]*2
    monkeypatch.setattr(route, 'stable_map_pose', lambda *args: (xy, 0., math.radians(yaw)))
    monkeypatch.setattr(route, 'finish_waypoint', lambda *a, **kw: pytest.fail('No separate arrival Spin'))
    assert route.run_waypoints(nav, points, 60, ROOT/'behavior_trees') == (0 if success else 1)
    assert len(nav.goals) == (2 if success else 1)
    assert all('navigate_precision_forward.xml' in str(tree) for _, tree in nav.goals)
    assert not nav.spin_requests


def test_failed_precision_navigation_never_retries(monkeypatch):
    nav = FakeNavigator(results=[route.TaskResult.FAILED])
    nav.nav2_precision_pose = True
    nav.verify_precision_controller = lambda: None
    monkeypatch.setattr(route, 'stable_map_pose', lambda *a: pytest.fail('Failed navigation'))
    points = [dict(x=0., y=0., yaw=0., mode='forward')]*2
    assert route.run_waypoints(nav, points, 60, ROOT/'behavior_trees') == 1
    assert len(nav.goals) == 1


def test_initial_departure_spin_is_preserved(monkeypatch):
    nav = FakeNavigator()
    nav.nav2_precision_pose = True
    nav.verify_precision_controller = lambda: None
    monkeypatch.setattr(route, 'stable_map_pose', lambda *a: (0.,0.,0.))
    points = [dict(x=0., y=0., yaw=0., mode='forward')]*2
    assert route.run_waypoints(nav, points, 60, ROOT/'behavior_trees', pre_turn_angle_deg=180) == 0
    assert len(nav.spin_requests) == 1
    assert nav.spin_requests[0][0] == pytest.approx(math.pi)
