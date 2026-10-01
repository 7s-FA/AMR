"""Requested sequence: initial departure, Nav2 position, stop, yaw at each point."""
import math
import pytest
from test_directional_waypoints import FakeNavigator, ROOT, route

POINTS = [dict(x=.42, y=.04, yaw=90., mode='forward'),
          dict(x=.44, y=.36, yaw=90., mode='forward')]


def navigator(monkeypatch):
    nav = FakeNavigator()
    nav.nav2_position_then_yaw = True
    nav.final_yaw_tolerance = math.radians(3)
    def pose(timeout=2):
        goal = nav.goals[-1][0].pose
        return goal.position.x, goal.position.y, math.pi/2 if nav.in_spin else 0.
    nav.current_map_pose = pose
    nav.align_for_travel = lambda *a: pytest.fail('No pre-travel spin')
    for name in ('terminal_approach', 'staged_final_approach'):
        monkeypatch.setattr(route, name, lambda *a: pytest.fail('Nav2 owns all translation'))
    return nav


def test_each_coordinate_is_reached_before_yaw_and_next_goal(monkeypatch):
    nav = navigator(monkeypatch)
    assert route.run_waypoints(nav, POINTS, 60, ROOT/'behavior_trees') == 0
    actions = [event for event in nav.events if event in ('navigate_forward', 'spin')]
    assert actions == ['navigate_forward', 'spin', 'navigate_forward', 'spin']
    assert len(nav.goals) == 2
    for pose, _ in nav.goals:
        assert pose.pose.position.x in (.42, .44)  # no offset staging goal
    assert len(nav.spin_requests) == 2


def test_departure_turn_is_preserved_once(monkeypatch):
    nav = navigator(monkeypatch)
    departures = []
    nav.pre_backup_timed = lambda d, s, deadline: departures.append((d, s))
    assert route.run_waypoints(nav, POINTS, 60, ROOT/'behavior_trees',
                               pre_backup_distance=.15, pre_backup_speed=.05,
                               pre_backup_open_loop=True, pre_turn_angle_deg=180) == 0
    assert departures == [(.15, .05)]
    actions = [event for event in nav.events if event in ('navigate_forward', 'spin')]
    assert actions == ['spin', 'navigate_forward', 'spin', 'navigate_forward', 'spin']
    assert nav.spin_requests[0][0] == pytest.approx(math.pi)


def test_no_early_handoff_and_yaw_required_at_intermediate_point(monkeypatch):
    nav = navigator(monkeypatch)
    waits, finishes = [], []
    monkeypatch.setattr(route, 'wait_for_task', lambda *a, **kw: waits.append(kw) or True)
    monkeypatch.setattr(route, 'finish_waypoint', lambda *a, **kw: finishes.append(kw) or True)
    assert route.run_waypoints(nav, POINTS, 60, ROOT/'behavior_trees') == 0
    assert waits == [{'waypoint': p, 'planned_route': True} for p in POINTS]
    assert finishes == [dict(is_final=False, require_yaw=True), dict(is_final=True, require_yaw=True)]


@pytest.mark.parametrize('failure', ['navigation', 'arrival', 'yaw'])
def test_failure_stops_without_next_goal_or_translation_retry(monkeypatch, failure):
    nav = navigator(monkeypatch)
    if failure == 'navigation':
        nav.results = [route.TaskResult.FAILED]
    elif failure == 'arrival':
        nav.current_map_pose = lambda timeout=2: (0., 0., 0.)
    else:
        nav.spin_result = route.TaskResult.FAILED
    assert route.run_waypoints(nav, POINTS, 60, ROOT/'behavior_trees') == 1
    assert len(nav.goals) == 1
