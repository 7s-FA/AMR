"""Terminal approach sequencing/geometry regression; never starts a ROS node."""
import math
from types import SimpleNamespace
import pytest
from test_directional_waypoints import route, FakeNavigator, ROOT


def terminal_nav(poses, stopped=True):
    n = FakeNavigator(stopped=stopped)
    n.xy_tolerance = .02
    n.map_poses = list(poses)
    n.drives = []
    def drive(**kw):
        n.in_spin = False
        n.drives.append(kw)
        return True
    n.driveOnHeading = drive
    return n


@pytest.mark.parametrize('mode,yaw,sign', [('forward', 0., 1), ('reverse', math.pi, -1)])
def test_signed_straight_goal_keeps_direction_and_stopping_margin(mode, yaw, sign):
    n = terminal_nav([(0., 0., yaw)])
    p = dict(x=.10, y=0., mode=mode)
    assert route.terminal_approach(n, p, route.time.monotonic()+30, 1)
    assert len(n.drives) == 1
    assert n.drives[0]['dist'] == pytest.approx(sign*.09)
    assert n.drives[0]['speed'] == sign*.03
    assert 0 < n.drives[0]['time_allowance'] <= 12
    assert not n.spin_requests
    assert 'stopped' in n.events


def test_recorded_failed_approach_is_aligned_before_driving():
    # Representative pose on the recorded failed straight row, 10cm out.
    p = dict(x=.42062385, y=.036671807, mode='reverse')
    start = (.325, .005, math.pi)
    desired = math.atan2(p['y']-start[1], p['x']-start[0])+math.pi
    n = terminal_nav([start, (start[0], start[1], desired)])
    assert route.terminal_approach(n, p, route.time.monotonic()+40, 1)
    assert len(n.spin_requests) == 1
    assert n.spin_requests[0][0] == pytest.approx(.319847, abs=.001)
    travel = -n.drives[0]['dist']
    end = (start[0]-travel*math.cos(desired), start[1]-travel*math.sin(desired))
    assert math.hypot(end[0]-p['x'], end[1]-p['y']) == pytest.approx(.01)
    # This is ideal geometry, not a claim of physical drive accuracy.


@pytest.mark.parametrize('distance', [.021, .05, .1, .2, .25])
def test_heading_budget_leaves_room_inside_two_cm(distance):
    tolerance = route.terminal_heading_tolerance(distance, .02)
    assert tolerance <= math.radians(7)
    travel = distance-.01
    residual = math.hypot(distance-travel*math.cos(tolerance), travel*math.sin(tolerance))
    assert residual < .02


@pytest.mark.parametrize('case', ['moving', 'outside', 'unaligned', 'no_time', 'already_arrived'])
def test_no_drive_without_stop_alignment_range_and_time(case):
    pose = (.0, .0, math.pi if case=='unaligned' else 0.)
    n = terminal_nav([pose]*3, stopped=case!='moving')
    p = dict(x=.3 if case=='outside' else .01 if case=='already_arrived' else .1, y=0.)
    deadline = route.time.monotonic()+(1 if case=='no_time' else 40)
    assert route.terminal_approach(n, p, deadline, 1) == (case=='already_arrived')
    assert not n.drives
    assert len(n.spin_requests) <= 2


def test_recomputes_distance_and_heading_after_spin():
    n = terminal_nav([(0, 0, math.pi), (.03, .01, math.atan2(-.01, .07))])
    assert route.terminal_approach(n, dict(x=.1, y=0.), route.time.monotonic()+40, 1)
    assert n.drives[0]['dist'] == pytest.approx(math.hypot(.07, .01)-.01)


@pytest.mark.parametrize('stop_ok', [True, False])
def test_handoff_requires_cancel_and_stop_confirmation(monkeypatch, stop_ok):
    n = FakeNavigator()
    n.isTaskComplete = lambda: False
    n.current_map_pose = lambda **kw: (.09, 0, 0)
    calls = []
    n.cancel_guarded_translation = lambda **kw: calls.append(kw) or stop_ok
    result = route.wait_for_task(n, route.time.monotonic()+30, 1,
                                waypoint=dict(x=0., y=0.), terminal_handoff=True)
    assert result is (route.TERMINAL_HANDOFF if stop_ok else False)
    assert calls == [dict(for_handoff=True)]


@pytest.mark.parametrize('terminal_ok,finish_ok', [(False, True), (True, False), (True, True)])
def test_route_terminal_failure_never_retries_or_advances(monkeypatch, terminal_ok, finish_ok):
    n = FakeNavigator()
    monkeypatch.setattr(route, 'wait_for_task', lambda *a, **kw: route.TERMINAL_HANDOFF)
    terminal_calls = []
    def terminal(*a):
        terminal_calls.append(a[2:])
        return terminal_ok
    monkeypatch.setattr(route, 'terminal_approach', terminal)
    def finish(*a, **kw):
        n.position_retry_requested = True
        return finish_ok
    monkeypatch.setattr(route, 'finish_waypoint', finish)
    points = [dict(x=.4,y=0.,yaw=0.,mode='reverse'), dict(x=.4,y=.3,yaw=90.)]
    assert route.run_waypoints(n, points, 30, ROOT/'behavior_trees') == (0 if terminal_ok and finish_ok else 1)
    assert len(n.goals) == (2 if terminal_ok and finish_ok else 1)
    assert len(terminal_calls) == len(n.goals)


def test_terminal_behavior_failure_propagates_without_retry():
    n = terminal_nav([(0, 0, 0)])
    n.results = [route.TaskResult.FAILED]
    assert not route.terminal_approach(n, dict(x=.1,y=0.), route.time.monotonic()+30, 1)
    assert len(n.drives) == 1


def test_terminal_goal_rejection_does_not_start_another_action():
    n = terminal_nav([(0, 0, 0)])
    n.driveOnHeading = lambda **kw: False
    assert not route.terminal_approach(n, dict(x=.1,y=0.), route.time.monotonic()+30, 1)
    assert not n.spin_requests


def test_terminal_keeps_stale_pose_and_overshoot_guard(monkeypatch):
    n = terminal_nav([(0, 0, 0)])
    calls = []
    monkeypatch.setattr(route, 'wait_for_task', lambda *a, **kw: calls.append(kw) or False)
    p = dict(x=.1, y=0.)
    assert not route.terminal_approach(n, p, route.time.monotonic()+30, 1)
    assert calls == [dict(waypoint=p)]  # No handoff/recovery inside this action.


@pytest.mark.parametrize('case', ['ok', 'wrong_frame', 'wrong_plugin', 'no_prediction', 'missing_server'])
def test_live_behavior_preflight_rejects_incompatible_configuration(monkeypatch, case):
    from rcl_interfaces.msg import ParameterValue
    values = [ParameterValue(type=9,string_array_value=['drive_on_heading']),
              ParameterValue(type=4,string_value='wrong' if case=='wrong_plugin' else 'nav2_behaviors::DriveOnHeading'),
              ParameterValue(type=4,string_value='burger1/odom' if case=='wrong_frame' else 'burger1/map'),
              ParameterValue(type=4,string_value='burger1/base_link'),
              ParameterValue(type=3,double_value=0. if case=='no_prediction' else 2.)]
    future = SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(values=values))
    client = SimpleNamespace(wait_for_service=lambda **kw:True,call_async=lambda req:future)
    destroyed=[]
    action = SimpleNamespace(wait_for_server=lambda **kw:case!='missing_server')
    n=SimpleNamespace(frame_prefix='burger1/',_verify_departure_pipeline=lambda:None,
                      create_client=lambda *a:client,destroy_client=lambda c:destroyed.append(c),
                      spin_client=action,drive_on_heading_client=action)
    monkeypatch.setattr(route.rclpy,'spin_until_future_complete',lambda *a,**kw:None)
    if case=='ok':
        route.WaypointNavigator.verify_terminal_behavior(n)
    else:
        with pytest.raises(RuntimeError):
            route.WaypointNavigator.verify_terminal_behavior(n)
    assert destroyed==[client]
