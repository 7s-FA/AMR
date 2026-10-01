"""Offline approach watchdog tests; no ROS nodes or motion publishers are created."""
from types import SimpleNamespace

import pytest

from test_directional_waypoints import FakeNavigator, ROOT, route


def test_far_goal_can_recede_without_arming():
    guard = route.ApproachOvershootGuard()
    for t, distance in enumerate([1., .3, .11, .4, .2]):
        assert not guard.update(distance, float(t))


def test_near_goal_recession_requires_continuous_hold():
    guard = route.ApproachOvershootGuard()
    assert not guard.update(.09, 0.)
    assert not guard.update(.025, .1)
    assert not guard.update(.06, .2)
    assert not guard.update(.06, .49)
    assert guard.update(.06, .51)


def test_return_toward_goal_resets_hold_and_tracks_new_best():
    guard = route.ApproachOvershootGuard()
    for distance, t in [(.08, 0.), (.03, .1), (.065, .2), (.04, .4),
                        (.02, .5), (.055, .6), (.055, .89)]:
        assert not guard.update(distance, t)
    assert guard.update(.055, .91)
    assert guard.best_distance == .02
    fresh = route.ApproachOvershootGuard()
    assert not fresh.update(.055, 5.)  # The next goal gets independent state.


@pytest.mark.parametrize('distance', [float('nan'), float('inf'), -.01])
def test_invalid_distance_is_not_treated_as_safe(distance):
    with pytest.raises(ValueError):
        route.ApproachOvershootGuard().update(distance, 1.)


@pytest.mark.parametrize('mode', ['forward', 'reverse'])
def test_guard_cancels_route_without_success_retry_or_next_goal(monkeypatch, mode):
    # Exercise the guard independently of the earlier planned 10cm handoff.
    monkeypatch.setattr(route, 'TERMINAL_APPROACH_DISTANCE', 0.0)
    clock = [0.]
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(route.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0]+dt))
    nav = FakeNavigator()
    nav.xy_tolerance = .02
    distances = iter([.2, .09, .027] + [.065]*10)
    canceled = []

    def complete():
        clock[0] += .1
        return bool(canceled)

    def pose(timeout):
        return next(distances), 0., 0.

    def stop():
        canceled.append(True)
        nav.events.extend(['guard_cancel', 'stop_confirmed'])
        return True

    nav.isTaskComplete = complete
    nav.current_map_pose = pose
    nav.cancel_guarded_translation = stop
    points = [dict(x=0., y=0., yaw=0., mode=mode), dict(x=1., y=0., yaw=0.)]
    assert route.run_waypoints(nav, points, 30, ROOT/'behavior_trees') == 1
    assert canceled == [True]
    assert len(nav.goals) == 1 and not nav.spin_requests
    assert 'result' not in nav.events
    assert nav.xy_tolerance == .02


def test_unavailable_localization_cancels_instead_of_disabling_guard(monkeypatch):
    nav = FakeNavigator()
    nav.isTaskComplete = lambda: False
    def missing(timeout):
        raise RuntimeError('stale TF')
    nav.current_map_pose = missing
    nav.cancel_guarded_translation = lambda: nav.events.append('guard_cancel')
    assert not route.wait_for_task(nav, route.time.monotonic()+30, 1,
                                   waypoint=dict(x=0., y=0.))
    assert nav.events == ['guard_cancel']


def test_spin_wait_does_not_read_map_or_apply_translation_guard(monkeypatch):
    nav = FakeNavigator()
    completed = iter([False, True])
    nav.isTaskComplete = lambda: next(completed)
    nav.current_map_pose = lambda **kw: pytest.fail('Spin must not use approach guard')
    monkeypatch.setattr(route.time, 'sleep', lambda dt: None)
    assert route.wait_for_task(nav, route.time.monotonic()+30, 1)


def test_normal_approach_completes_without_cancel(monkeypatch):
    nav = FakeNavigator()
    distances = [.2, .09, .04, .01]
    nav.isTaskComplete = lambda: not distances
    nav.current_map_pose = lambda **kw: (distances.pop(0), 0., 0.)
    nav.cancel_guarded_translation = lambda: pytest.fail('Normal approach canceled')
    monkeypatch.setattr(route.time, 'sleep', lambda dt: None)
    assert route.wait_for_task(nav, route.time.monotonic()+30, 1,
                               waypoint=dict(x=0., y=0.))


@pytest.mark.parametrize('case', ['success', 'no_ack', 'still_active', 'moving', 'request_error'])
def test_cancellation_is_bounded_and_checks_actual_stop(monkeypatch, case):
    clock = [0.]
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    pending = SimpleNamespace(done=lambda: case != 'no_ack', result=lambda: object())
    monkeypatch.setattr(route.rclpy, 'spin_until_future_complete',
                        lambda n, f, timeout_sec: clock.__setitem__(0, clock[0]+timeout_sec))
    def cancel():
        if case == 'request_error':
            raise RuntimeError('cancel transport unavailable')
        return pending
    def complete():
        clock[0] += .1
        return case != 'still_active'
    stop_checks = []
    def stopped(timeout):
        stop_checks.append(timeout)
        return case != 'moving'
    nav = SimpleNamespace(goal_handle=SimpleNamespace(cancel_goal_async=cancel),
                          isTaskComplete=complete, wait_until_stopped=stopped,
                          error=lambda text: None, info=lambda text: None)
    assert route.WaypointNavigator.cancel_guarded_translation(nav) == (case == 'success')
    assert stop_checks == [3.0]
    assert clock[0] < 6.2
