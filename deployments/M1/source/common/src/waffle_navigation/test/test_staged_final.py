"""Final approach regression against the 2026-09-29 rotation failure."""
import math
import pytest
from test_directional_waypoints import FakeNavigator, ROOT, route


POINT = dict(x=.43906513, y=.360654172, yaw=90.760873754, mode='forward')


def nav_fixture(monkeypatch, lateral=0., final_error=.006, yaw_error=0., initial_yaw=None):
    nav = FakeNavigator()
    nav.xy_tolerance = .02
    nav.final_yaw_tolerance = math.radians(3)
    nav.final_staging_distance = .18
    yaw = math.radians(POINT['yaw'])
    stage = (POINT['x']-.18*math.cos(yaw)-lateral*math.sin(yaw),
             POINT['y']-.18*math.sin(yaw)+lateral*math.cos(yaw), yaw)
    end = (POINT['x']-final_error*math.cos(yaw),
           POINT['y']-final_error*math.sin(yaw), yaw+math.radians(yaw_error))
    poses = ([(*stage[:2], initial_yaw), stage, end] if initial_yaw is not None else [stage, end])
    monkeypatch.setattr(route, 'stable_map_pose', lambda nav, deadline: poses.pop(0))
    nav.drives = []
    nav.driveOnHeading = lambda **args: nav.drives.append(args) or True
    return nav


def run(nav):
    return route.staged_final_approach(nav, POINT, route.time.monotonic()+60, 2,
                                     ROOT/'behavior_trees/navigate_forward.xml')


def test_align_at_stage_then_forward_without_final_spin(monkeypatch):
    nav = nav_fixture(monkeypatch, initial_yaw=math.radians(12))
    assert run(nav)
    assert len(nav.goals) == len(nav.drives) == len(nav.spin_requests) == 1
    assert nav.spin_requests[0][0] == pytest.approx(math.radians(78.760873754))
    assert nav.drives[0]['dist'] == pytest.approx(.174)
    assert nav.drives[0]['speed'] == .03
    pose = nav.goals[0][0].pose.position
    assert math.hypot(pose.x-POINT['x'], pose.y-POINT['y']) == pytest.approx(.18)


def test_recorded_three_point_two_cm_error_still_fails_without_rotation(monkeypatch):
    nav = nav_fixture(monkeypatch, final_error=.032, yaw_error=-2.2)
    assert not run(nav)
    assert len(nav.drives) == 1
    assert not nav.spin_requests


def test_lateral_miss_does_not_turn_toward_near_goal(monkeypatch):
    nav = nav_fixture(monkeypatch, lateral=.024)
    assert not run(nav)
    assert not nav.drives and not nav.spin_requests


def test_yaw_failure_does_not_trigger_a_new_spin_at_endpoint(monkeypatch):
    nav = nav_fixture(monkeypatch, yaw_error=4)
    assert not run(nav)
    assert not nav.spin_requests


def test_failed_stage_navigation_never_drives(monkeypatch):
    nav = nav_fixture(monkeypatch)
    nav.results = [route.TaskResult.FAILED]
    assert not run(nav)
    assert not nav.drives and not nav.spin_requests


@pytest.mark.parametrize('unstable', [False, True])
def test_map_stability_has_bounded_wait(monkeypatch, unstable):
    clock = [0.]
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(route, 'spin_for', lambda nav, duration: clock.__setitem__(0, clock[0]+duration))
    nav = FakeNavigator()
    calls = [0]
    def pose(timeout):
        calls[0] += 1
        return (.02*(calls[0] % 2) if unstable else .001, 0., 0.)
    nav.current_map_pose = pose
    if unstable:
        with pytest.raises(RuntimeError, match='안정되지'):
            route.stable_map_pose(nav, 10)
    else:
        assert route.stable_map_pose(nav, 10) == (.001, 0., 0.)
    assert .5 <= clock[0] < 4.1
