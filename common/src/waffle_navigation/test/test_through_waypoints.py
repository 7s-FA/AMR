"""Offline route/action simulation: never creates a ROS node or motion publisher."""
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from test_directional_waypoints import FakeNavigator, ROOT, route

POINTS = [dict(x=.42, y=.04, yaw=90., mode='forward'),
          dict(x=.47, y=.39, yaw=90., mode='forward')]


class Progress:
    def __init__(self, completed=0):
        self.state = dict(completed=completed, deadline=10**12, backup_claimed=False,
                          backup_done=True, turn_done=True)
        self.saved = []

    def complete(self, index):
        assert index == self.state['completed'] + 1
        self.state['completed'] = index
        self.saved.append(index)


class ThroughNavigator(FakeNavigator):
    def __init__(self):
        super().__init__()
        self.nav2_position_then_yaw = True
        self.through_intermediate = True
        self.frame_prefix = 'burger1/'
        self.feedback = None
        self.through_goals = []
        self.samples = []
        self.next_samples = [(2, (0., 0., 0.)), (1, (.42, .04, 0.)),
                             (1, (.47, .39, 1.57))]
        self.position = (.47, .39, 1.57)
        self.nav_through_poses_client = SimpleNamespace(wait_for_server=lambda **kw: True)

    def getFeedback(self):
        return self.feedback

    def goThroughPoses(self, poses, behavior_tree):
        assert self.feedback is None  # no stale feedback from a prepared worker
        self.through_goals.append((list(poses), behavior_tree))
        self.events.append('through')
        self.samples = list(self.next_samples)
        return self.accepted

    def isTaskComplete(self):
        if not self.samples:
            return True
        remaining, self.position = self.samples.pop(0)
        self.feedback = SimpleNamespace(number_of_poses_remaining=remaining)
        return False

    def current_map_pose(self, timeout=2.):
        return self.position

    def getResult(self):
        return self.results[0]


@pytest.fixture
def nav(monkeypatch):
    monkeypatch.setattr(route.time, 'sleep', lambda dt: None)
    return ThroughNavigator()


def run(nav):
    return route.run_waypoints(nav, POINTS, 60, ROOT/'behavior_trees')


def test_route_uses_one_action_and_only_final_stop_and_yaw(nav, monkeypatch):
    nav.route_progress = Progress()
    def finish(n, point, deadline, index, **kw):
        assert nav.events.count('stopped') == 1  # initial stop, no intermediate stop
        assert nav.route_progress.saved == [1]
        assert point == POINTS[-1] and index == 2
        assert kw == dict(is_final=True, require_yaw=True)
        nav.events.extend(['final_stop', 'final_yaw'])
        return True
    monkeypatch.setattr(route, 'finish_waypoint', finish)
    assert run(nav) == 0
    assert len(nav.through_goals) == 1 and not nav.goals and not nav.spin_requests
    assert len(nav.through_goals[0][0]) == 2
    assert nav.route_progress.saved == [1, 2]
    assert 'cancel' not in nav.events


def test_final_position_and_heading_checks_are_still_real(nav):
    assert run(nav) == 0
    assert nav.events.count('stopped') >= 2
    assert not nav.goals  # no individual goToPose


def test_missing_via_point_never_finishes_even_if_nav2_succeeds(nav, monkeypatch):
    nav.route_progress = Progress()
    nav.next_samples = [(2, (.47, .39, 1.57))]
    monkeypatch.setattr(route, 'finish_waypoint', lambda *a, **kw: pytest.fail('Skipped via point'))
    assert run(nav) == 1
    assert nav.route_progress.saved == []


def test_failed_route_checkpoints_passed_point_but_not_final(nav):
    nav.route_progress = Progress()
    nav.results = [route.TaskResult.FAILED]
    assert run(nav) == 1
    assert nav.route_progress.saved == [1]


def test_external_retry_sends_only_unfinished_suffix(nav):
    nav.route_progress = Progress(completed=1)
    nav.next_samples = [(1, (.47, .39, 1.57))]
    assert run(nav) == 0
    poses = nav.through_goals[0][0]
    assert len(poses) == 1 and poses[0].pose.position.x == .47
    assert nav.route_progress.saved == [2]


def test_final_arrival_failure_does_not_mark_order_complete(nav):
    nav.route_progress = Progress()
    nav.next_samples = [(1, (.47, .30, 1.57))]
    assert run(nav) == 1
    assert nav.route_progress.saved == [1]


@pytest.mark.parametrize('feedback', [0, 3, -1, None, True])
def test_invalid_feedback_cancels_instead_of_skipping_points(nav, feedback):
    nav.next_samples = [(feedback, (.42, .04, 0.))]
    assert run(nav) == 1
    assert 'cancel' in nav.events


def test_localization_failure_cancels(nav):
    def missing(**kw):
        raise RuntimeError('stale TF')
    nav.current_map_pose = missing
    assert run(nav) == 1
    assert 'cancel' in nav.events


def test_deadline_cancels_through_action(nav, monkeypatch):
    clock = [0.]
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(route.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0]+dt))
    nav.isTaskComplete = lambda: False
    assert not route.wait_for_task(nav, .11, 2, waypoint=POINTS[-1], planned_route=True,
                                   passage=route.ThroughPassage(2))
    assert 'cancel' in nav.events


def test_missing_through_server_does_not_send_motion(nav):
    nav.nav_through_poses_client.wait_for_server = lambda **kw: False
    assert run(nav) == 1
    assert nav.through_goals == []


def test_rejected_goal_has_no_retry_or_final_check(nav):
    nav.accepted = False
    assert run(nav) == 1
    assert len(nav.through_goals) == 1 and 'final_yaw' not in nav.events


def test_already_completed_route_sends_nothing(nav):
    nav.route_progress = Progress(completed=2)
    assert run(nav) == 0
    assert not nav.through_goals


def test_single_point_stays_on_existing_go_to_pose_path(nav, monkeypatch):
    monkeypatch.setattr(route, 'wait_for_task', lambda *a, **kw: True)
    monkeypatch.setattr(route, 'finish_waypoint', lambda *a, **kw: True)
    assert route.run_waypoints(nav, POINTS[-1:], 60, ROOT/'behavior_trees') == 0
    assert len(nav.goals) == 1 and not nav.through_goals


def test_reverse_route_rejected_before_departure(nav):
    points = [dict(POINTS[0], mode='reverse'), POINTS[1]]
    with pytest.raises(RuntimeError, match='전진'):
        route.run_waypoints(nav, points, 60, ROOT/'behavior_trees', pre_backup_distance=.1)
    assert not nav.events


def test_bt_has_continuous_follow_and_no_recovery_or_wait():
    tree = ET.parse(ROOT/'behavior_trees/navigate_through_forward.xml')
    pipeline = tree.find('.//PipelineSequence')
    assert [node.tag for node in pipeline] == ['RemovePassedGoals', 'RateController', 'FollowPath']
    assert float(pipeline[0].attrib['radius']) == .06
    assert float(pipeline[1].attrib['hz']) == 1.
    follow = pipeline[2]
    assert follow.attrib['controller_id'] == 'FollowPositionForward'
    assert follow.attrib['goal_checker_id'] == 'position_goal_checker'
    assert len(tree.findall('.//ComputePathThroughPoses')) == 1
    for tag in ('Spin', 'Wait', 'BackUp', 'RecoveryNode', 'ComputePathToPose'):
        assert not tree.findall('.//' + tag)


def test_progress_never_moves_backwards():
    passage = route.ThroughPassage(3)
    assert not passage.update(SimpleNamespace(number_of_poses_remaining=2))
    with pytest.raises(RuntimeError):
        passage.update(SimpleNamespace(number_of_poses_remaining=3))


@pytest.mark.parametrize('fault', ['path', 'overshoot'])
def test_through_route_keeps_path_and_final_overshoot_guards(nav, monkeypatch, fault):
    clock = [0.]
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(route.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0]+.2))
    if fault == 'path':
        nav.current_plan_offset = lambda *a: .2
        nav.next_samples = [(2, (0., 0., 0.))] * 20
    else:
        nav.next_samples = [(1, (.47, .38, 1.57))] + [(1, (.47, .32, 1.57))] * 20
    assert run(nav) == 1
    assert 'cancel' in nav.events
    assert nav.navigation_retry_reason == ('planned_path_deviation' if fault == 'path' else 'target_overshoot')


def test_internal_retry_skips_passed_point_and_aligns_only_on_retry(nav, monkeypatch):
    nav.navigation_retries = 1
    nav.route_progress = Progress()
    calls = []
    def wait(n, deadline, index, **kw):
        calls.append(kw)
        kw['passage'].update(SimpleNamespace(number_of_poses_remaining=1))
        n.feedback = SimpleNamespace(number_of_poses_remaining=1)
        n.navigation_retry_reason = 'planned_path_deviation'
        return len(calls) > 1
    recoveries = []
    monkeypatch.setattr(route, 'wait_for_task', wait)
    monkeypatch.setattr(route, 'prepare_navigation_retry', lambda *args: recoveries.append(args))
    alignments = []
    nav.align_for_travel = lambda p, *args: alignments.append(p) or True
    assert run(nav) == 0
    assert [len(g[0]) for g in nav.through_goals] == [2, 1]
    assert len(recoveries) == 1 and alignments == [POINTS[-1]]
    assert nav.route_progress.saved == [1, 2]


def test_retry_loss_of_ownership_cannot_send_another_goal(nav, monkeypatch, tmp_path):
    import json
    nav.navigation_retries = 1
    nav.retry_gate_path = tmp_path/'gate.json'
    nav.retry_gate_path.write_text(json.dumps(dict(estop=True, goal_id='old', heartbeat=route.time.monotonic())))
    def fail(*args, **kw):
        nav.navigation_retry_reason = 'planned_path_deviation'
        return False
    monkeypatch.setattr(route, 'wait_for_task', fail)
    with pytest.raises(RuntimeError, match='제어권'):
        run(nav)
    assert len(nav.through_goals) == 1
