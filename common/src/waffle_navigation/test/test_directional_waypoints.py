"""Offline route sequencing and direction contract tests; no ROS graph required."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('nav2_waypoints', ROOT / 'scripts/nav2_waypoints.py')
route = importlib.util.module_from_spec(spec)
spec.loader.exec_module(route)


def test_way12_terminal_handoff_uses_route_distance():
    class NearGoal:
        terminal_approach_distance = 0.06

        def __init__(self):
            self.checks = 0
            self.canceled = False

        def current_map_pose(self, timeout=2.0):
            return (0.055, 0.0, 0.0)

        def isTaskComplete(self):
            self.checks += 1
            return self.checks > 1

        def cancel_guarded_translation(self, for_handoff=False):
            self.canceled = for_handoff
            return True

        def getResult(self):
            return route.TaskResult.SUCCEEDED

    nav = NearGoal()
    result = route.wait_for_task(nav, route.time.monotonic() + 2, 2,
                                 waypoint={'x': 0.0, 'y': 0.0},
                                 terminal_handoff=True)
    assert result is route.TERMINAL_HANDOFF
    assert nav.canceled


@pytest.mark.parametrize('mode', ['forward', 'reverse', None])
def test_load_modes_and_legacy_coordinates(tmp_path, mode):
    point = {'x': 1.0, 'y': 2.0, 'yaw': 90.0}
    if mode is not None:
        point['mode'] = mode
    path = tmp_path / 'route.yaml'
    path.write_text(yaml.safe_dump({'waypoints': [point]}))
    assert route.load_waypoints(path)[0]['mode'] == (mode or 'forward')


@pytest.mark.parametrize('point', [
    {'x': 0, 'y': 0, 'yaw': 0, 'mode': 'backward'},
    {'x': 0, 'y': 0, 'yaw': 0, 'mode': ['reverse']},
    {'x': 0, 'y': 0, 'yaw': float('nan')},
    {'x': True, 'y': 0, 'yaw': 0},
    {'x': 0, 'y': 0, 'yaw': 0, 'direction': 'reverse'},
])
def test_invalid_route_is_rejected(tmp_path, point):
    path = tmp_path / 'route.yaml'
    path.write_text(yaml.safe_dump({'waypoints': [point]}))
    with pytest.raises(ValueError):
        route.load_waypoints(path)


class FakeNavigator:
    def current_plan_offset(self, waypoint, pose, goal_started):
        # No plan received in generic sequencing/overshoot tests.
        return None

    def info(self, message):
        print(message)

    def error(self, message):
        print(message)

    def __init__(self, results=None, stopped=True, accepted=True):
        self.events = []
        self.results = results or [route.TaskResult.SUCCEEDED] * 2
        self.stopped = stopped
        self.accepted = accepted
        self.goals = []
        self.xy_tolerance = 0.05
        self.intermediate_xy_tolerance = 0.10
        self.yaw_tolerance = 0.15
        self.map_poses = []
        self.in_spin = False
        self.spin_result = route.TaskResult.SUCCEEDED
        self.spin_requests = []

    def get_clock(self):
        from builtin_interfaces.msg import Time
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))

    def waitUntilNav2Active(self):
        self.events.append('ready')

    def verify_controllers(self, modes):
        self.events.append(('verified', modes))

    def verify_terminal_behavior(self):
        pass

    def cancel_guarded_translation(self, for_handoff=False):
        self.cancelTask()
        return self.stopped

    def wait_until_stopped(self, timeout=5.0):
        self.events.append('stopped')
        return self.stopped

    def align_for_travel(self, waypoint, deadline, index):
        return True

    def goToPose(self, pose, behavior_tree):
        self.in_spin = False
        self.goals.append((pose, behavior_tree))
        self.events.append(Path(behavior_tree).stem)
        return self.accepted

    def isTaskComplete(self):
        return True

    def getResult(self):
        self.events.append('result')
        if self.in_spin:
            return self.spin_result
        return self.results[min(len(self.goals) - 1, len(self.results)-1)]

    def cancelTask(self):
        self.events.append('cancel')

    def current_map_pose(self, timeout=2.0):
        self.events.append('map_pose')
        if self.map_poses:
            return self.map_poses.pop(0)
        pose = self.goals[-1][0].pose
        q = pose.orientation
        yaw = route.math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        return pose.position.x, pose.position.y, yaw

    def spin(self, spin_dist, time_allowance):
        self.in_spin = True
        self.events.append('spin')
        self.spin_requests.append((spin_dist, time_allowance))
        return True


POINTS = [{'x': -0.5, 'y': 0.0, 'yaw': 0.0, 'mode': 'reverse'},
          {'x': 0.5, 'y': 0.0, 'yaw': 0.0, 'mode': 'forward'}]


def test_namespaced_goals_and_odometry_reject_another_robot():
    nav = FakeNavigator()
    nav.frame_prefix = 'burger2/'
    assert route.make_goal_pose_list(nav, POINTS)[0].header.frame_id == 'burger2/map'
    nav.odom_message = route.Odometry()
    nav.odom_message.pose.pose.orientation.w = 1.0
    nav.odom_received_at = route.time.monotonic()
    nav._fresh_stamp = lambda *args: True
    nav.odom_message.header.frame_id = 'burger2/odom'
    nav.odom_message.child_frame_id = 'burger2/base_footprint'
    assert route.WaypointNavigator._departure_pose(nav) == (0.0, 0.0, 0.0)
    nav.odom_message.header.frame_id = 'burger1/odom'
    with pytest.raises(RuntimeError, match='좌표계'):
        route.WaypointNavigator._departure_pose(nav)


def test_other_robot_actions_do_not_block_namespaced_departure():
    assert route.action_in_namespace('/burger2/navigate_to_pose/_action/status', '/burger2')
    assert not route.action_in_namespace('/burger1/navigate_to_pose/_action/status', '/burger2')
    assert not route.action_in_namespace('/burger20/navigate_to_pose/_action/status', '/burger2')
    assert not route.action_in_namespace('/navigate_to_pose/_action/status', '/burger2')


def run(nav, timeout=300):
    return route.run_waypoints(nav, POINTS, timeout, ROOT / 'behavior_trees')


def test_reverse_then_forward_waits_for_success_and_stop():
    nav = FakeNavigator()
    assert run(nav) == 0
    assert nav.events == ['ready', ('verified', {'forward', 'reverse'}),
                          'stopped', 'navigate_reverse', 'result',
                          'stopped', 'map_pose', 'map_pose',
                          'stopped', 'navigate_forward', 'result',
                          'stopped', 'map_pose', 'map_pose']
    assert nav.goals[0][0].pose.orientation.w == 1.0  # reverse does not add pi to yaw


@pytest.mark.parametrize('result', [route.TaskResult.FAILED, route.TaskResult.CANCELED])
def test_failed_leg_never_dispatches_next_waypoint(result):
    nav = FakeNavigator(results=[result])
    assert run(nav) == 1
    assert len(nav.goals) == 1


def test_missing_stop_confirmation_prevents_motion():
    nav = FakeNavigator(stopped=False)
    assert run(nav) == 1
    assert nav.goals == []


def test_rejected_goal_stops_route():
    nav = FakeNavigator(accepted=False)
    assert run(nav) == 1
    assert len(nav.goals) == 1


def test_timeout_cancels_active_goal_without_next_leg(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    nav = FakeNavigator()

    def complete():
        clock[0] = 2.0
        return 'cancel' in nav.events

    nav.isTaskComplete = complete
    assert run(nav, timeout=1.0) == 1
    assert nav.events[-1] == 'cancel'
    assert len(nav.goals) == 1


@pytest.mark.parametrize('case, expected', [('fresh', True), ('stale', False), ('moving', False)])
def test_stop_gate_requires_fresh_stationary_odometry(monkeypatch, case, expected):
    clock = [1.0]
    fake = SimpleNamespace(odom_received_at=0.0, odom_stopped=True)
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])

    def spin(node, timeout_sec):
        clock[0] += 0.1
        if case != 'stale':
            node.odom_received_at = clock[0]
        node.odom_stopped = case != 'moving'

    monkeypatch.setattr(route.rclpy, 'spin_once', spin)
    assert route.WaypointNavigator.wait_until_stopped(fake, timeout=1.5) == expected


def test_both_trees_use_matching_sign_restricted_controllers():
    config = yaml.safe_load((ROOT / 'config/nav2_burger_params.yaml').read_text())
    params = config['controller_server']['ros__parameters']
    for mode, name in route.CONTROLLERS.items():
        tree = ET.parse(ROOT / f'behavior_trees/navigate_{mode}.xml')
        assert tree.find('.//FollowPath').attrib['controller_id'] == name
        assert tree.find('.//FollowPath').attrib['goal_checker_id'] == 'position_goal_checker'
        assert tree.find('.//Spin') is None and tree.find('.//BackUp') is None
        assert name in params['controller_plugins']
        controller = params[name]
        low, high = controller['min_vel_x'], controller['max_vel_x']
        assert (low == 0 < high) if mode == 'forward' else (low < high == 0)
        assert 'BaseObstacle' in controller['critics']
        assert 'RotateToGoal' not in controller['critics']
    smoother = config['velocity_smoother']['ros__parameters']
    assert smoother['min_velocity'][0] < 0 < smoother['max_velocity'][0]


def test_installed_dwb_samples_exact_zero_for_final_rotation(tmp_path):
    """Exercise the actual DWB iterator: 20 reverse samples previously missed zero."""
    compiler = shutil.which('c++')
    assert compiler, 'C++ compiler required for the installed DWB regression test'
    prefix = Path(get_package_prefix('dwb_plugins'))
    includes = [prefix / 'include', prefix / 'include/dwb_plugins']
    binary = tmp_path / 'velocity_samples'
    subprocess.run([compiler, *[f'-I{path}' for path in includes],
                    str(ROOT / 'test/velocity_samples.cpp'), '-o', str(binary)], check=True)
    config = yaml.safe_load((ROOT / 'config/nav2_burger_params.yaml').read_text())
    params = config['controller_server']['ros__parameters']
    for controller in route.CONTROLLERS.values():
        settings = params[controller]
        subprocess.run([str(binary), str(settings['min_vel_x']),
                        str(settings['max_vel_x']), str(settings['vx_samples'])], check=True)


@pytest.mark.parametrize('already_received', [False, True])
def test_rviz_initial_pose_is_never_overwritten(monkeypatch, already_received):
    from unittest.mock import Mock
    nav = SimpleNamespace(initial_pose_received=already_received, info=Mock(),
                          _setInitialPose=Mock())
    spins = []

    def spin(node, timeout_sec):
        spins.append(True)
        # Receive the AMCL pose that came from the user's RViz estimate.
        node.initial_pose_received = True

    monkeypatch.setattr(route.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(route.rclpy, 'spin_once', spin)
    route.WaypointNavigator._waitForInitialPose(nav)
    nav._setInitialPose.assert_not_called()
    assert len(spins) == (0 if already_received else 1)


def test_failure_reports_nav2_error_code_and_message(capsys):
    from unittest.mock import Mock
    nav = FakeNavigator(results=[route.TaskResult.FAILED])
    nav.goals.append(None)
    nav.result_future = Mock()
    nav.result_future.done.return_value = True
    nav.result_future.result.return_value = SimpleNamespace(
        result=SimpleNamespace(error_code=208, error_msg='No valid path'))
    route.report_failure(nav, 1)
    output = capsys.readouterr().out
    assert 'FAILED' in output and 'error_code=208' in output and 'No valid path' in output


def test_position_then_stop_then_shortest_yaw_rotation():
    nav = FakeNavigator()
    nav.map_poses = [(-0.5, 0.0, route.math.pi / 2), (-0.5, 0.0, 0.0)]
    assert run(nav) == 0
    assert nav.spin_requests[0][0] == pytest.approx(-route.math.pi / 2)
    events = nav.events
    spin = events.index('spin')
    assert events[spin - 2:spin] == ['stopped', 'map_pose']
    assert events[spin + 1:spin + 4] == ['result', 'stopped', 'map_pose']
    assert events.index('navigate_forward') > spin + 3


def test_alignment_failure_never_dispatches_next_waypoint():
    nav = FakeNavigator()
    nav.map_poses = [(-0.5, 0.0, route.math.pi / 2)]
    nav.spin_result = route.TaskResult.FAILED
    assert run(nav) == 1
    assert len(nav.goals) == 1


@pytest.mark.parametrize('stage', ['before_spin', 'after_spin'])
def test_position_drift_aborts_without_corrective_driving(stage):
    nav = FakeNavigator()
    if stage == 'before_spin':
        nav.map_poses = [(-0.39, 0.0, route.math.pi / 2)]
    else:
        nav.map_poses = [(-0.5, 0.0, route.math.pi / 2), (-0.39, 0.0, 0.0)]
    assert run(nav) == 1
    assert len(nav.goals) == 1
    assert len(nav.spin_requests) == (stage == 'after_spin')


def test_unsatisfied_final_yaw_is_not_reported_as_success():
    nav = FakeNavigator()
    nav.map_poses = [(-0.5, 0.0, 1.0), (-0.5, 0.0, 0.5)]
    assert run(nav) == 1
    assert len(nav.goals) == 1


def test_intermediate_alignment_drift_from_report_allows_next_leg(capsys):
    nav = FakeNavigator()
    nav.map_poses = [(-0.466, 0.0, route.math.radians(112.1)),
                     (-0.425, 0.0, route.math.radians(4.0))]
    assert run(nav) == 0
    assert len(nav.goals) == 2
    assert len(nav.spin_requests) == 1
    output = capsys.readouterr().out
    assert '허용 0.100 m' in output
    assert '다음 웨이포인트로 진행' in output


@pytest.mark.parametrize('yaw_degrees, succeeds', [(11.1, True), (15.0, False)])
def test_configured_yaw_tolerance_handles_report_and_rejects_larger_error(capsys, yaw_degrees, succeeds):
    nav = FakeNavigator()
    config = yaml.safe_load((ROOT / 'config/nav2_burger_params.yaml').read_text())
    nav.yaw_tolerance = config['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance']
    nav.map_poses = [(-0.487, 0.0, route.math.radians(110.5)),
                     (-0.411, 0.0, route.math.radians(yaw_degrees))]
    assert run(nav) == (0 if succeeds else 1)
    assert len(nav.goals) == (2 if succeeds else 1)
    output = capsys.readouterr().out
    assert '허용 14.3°' in output
    if not succeeds:
        assert '최종 방향 허용오차' in output


@pytest.mark.parametrize('single_waypoint', [False, True])
def test_last_waypoint_retains_five_cm_after_rotation(single_waypoint):
    nav = FakeNavigator()
    points = POINTS[:1] if single_waypoint else POINTS
    x = points[-1]['x']
    nav.map_poses = ([] if single_waypoint else [(-0.5, 0.0, 0.0)]*2)
    nav.map_poses += [(x+0.034, 0.0, route.math.radians(112.1)),
                      (x+0.075, 0.0, route.math.radians(4.0))]
    assert route.run_waypoints(nav, points, 300, ROOT / 'behavior_trees') == 0
    assert len(nav.goals) == len(points) + 1


def test_intermediate_drift_without_spin_requires_reapproach():
    nav = FakeNavigator()
    nav.map_poses = [(-0.466, 0.0, 0.0), (-0.425, 0.0, 0.0)]
    assert run(nav) == 0
    assert len(nav.goals) == 3
    assert not nav.spin_requests


def test_intermediate_pre_turn_error_cannot_use_post_turn_allowance():
    nav = FakeNavigator()
    nav.yaw_tolerance = 0.25
    nav.map_poses = [(-0.445, 0.0, route.math.radians(113.9))]
    assert not route.finish_waypoint(nav, POINTS[0], route.time.monotonic()+30, 1, is_final=False)
    assert nav.position_retry_requested
    assert not nav.spin_requests


@pytest.mark.parametrize('stage,distance,succeeds', [
    ('before_spin',.019,True),('before_spin',.021,False),
    ('after_spin',.039,True),('after_spin',.041,False),
    ('no_spin',.019,True),('no_spin',.021,False)])
def test_only_completed_intermediate_spin_uses_larger_allowance(stage, distance, succeeds):
    nav = FakeNavigator()
    nav.xy_tolerance=.02
    nav.intermediate_xy_tolerance=.04
    if stage == 'before_spin':
        nav.map_poses = [(-0.5+distance, 0.0, route.math.pi/2), (-0.5, 0.0, 0.0)]
    else:
        nav.map_poses = [(-0.5, 0.0, route.math.pi/2 if stage == 'after_spin' else 0.0),
                         (-0.5+distance, 0.0, 0.0)]
    assert route.finish_waypoint(nav, POINTS[0], route.time.monotonic()+30, 1,
                                 is_final=False) is succeeds


def test_final_waypoint_does_not_inherit_intermediate_stop_tolerance():
    nav = FakeNavigator()
    nav.map_poses = [(-0.445, 0.0, route.math.pi/2)]
    assert route.run_waypoints(nav, POINTS[:1], 300, ROOT / 'behavior_trees') == 0
    assert len(nav.goals) == 2  # must reapproach, not accept 55 mm as final
    assert not nav.spin_requests


def test_yaw_wraparound_uses_short_rotation():
    point = {'x': 0.0, 'y': 0.0, 'yaw': -179.0}
    distance, angle = route.pose_errors(point, (0.0, 0.0, route.math.radians(179)))
    assert distance == 0.0
    assert route.math.degrees(angle) == pytest.approx(2.0)


def test_departure_happens_once_before_first_goal():
    nav = FakeNavigator()
    nav.pre_backup = lambda distance, speed, deadline: nav.events.append(('departure', distance, speed))
    assert route.run_waypoints(nav, POINTS, 300, ROOT / 'behavior_trees', 0.1, 0.05) == 0
    assert nav.events[2] == ('departure', 0.1, 0.05)
    assert sum(isinstance(e, tuple) and e[0] == 'departure' for e in nav.events) == 1


def test_departure_drains_multiple_sensor_callbacks_each_command_cycle(monkeypatch):
    clock, callbacks = [0.0], []
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(route.rclpy, 'ok', lambda: True)
    def spin(node, timeout_sec):
        callbacks.append(node)
        clock[0] += 0.001
    monkeypatch.setattr(route.rclpy, 'spin_once', spin)
    route.spin_for('node', 0.05)
    assert len(callbacks) >= 50


def test_departure_failure_never_sends_waypoint():
    nav = FakeNavigator()
    def fail(*args):
        raise RuntimeError('blocked')
    nav.pre_backup = fail
    with pytest.raises(RuntimeError, match='blocked'):
        route.run_waypoints(nav, POINTS, 300, ROOT / 'behavior_trees', 0.1, 0.05)
    assert not nav.goals


def test_departure_distance_is_measured_along_initial_body_direction():
    assert route.backup_progress((1, 2, route.math.pi/2), (1, 1.9, route.math.pi/2)) == pytest.approx(0.1)
    for pose in ((0.02, 0, 0), (-0.1, 0.03, 0), (-0.1, 0, 0.2)):
        with pytest.raises(RuntimeError):
            route.backup_progress((0, 0, 0), pose)


def scan_fixture():
    from sensor_msgs.msg import LaserScan
    return LaserScan(angle_min=0.0, angle_increment=route.math.pi/180,
                     range_min=0.12, range_max=3.5, ranges=[1.0]*360)


def test_departure_scan_checks_rear_obstacle_and_sensor_offset():
    scan = scan_fixture()
    route.check_rear_scan(scan, -0.032, 0, 0)
    # Wall ahead is not in the rear corridor (collision monitor remains active).
    scan.ranges[0] = 0.15
    route.check_rear_scan(scan, -0.032, 0, 0)
    scan.ranges[180] = 0.15
    with pytest.raises(RuntimeError, match='장애물'):
        route.check_rear_scan(scan, -0.032, 0, 0)
    # A scan mounted backwards must still check the robot's rear.
    scan = scan_fixture()
    scan.ranges[0] = 0.15
    with pytest.raises(RuntimeError, match='장애물'):
        route.check_rear_scan(scan, -0.032, 0, route.math.pi)


@pytest.mark.parametrize('invalid', [0.0, float('nan'), -float('inf')])
def test_departure_rejects_unobserved_rear_sector(invalid):
    scan = scan_fixture()
    for i in range(160, 200):
        scan.ranges[i] = invalid
    with pytest.raises(RuntimeError, match='관측'):
        route.check_rear_scan(scan, -0.032, 0, 0)


def test_departure_handles_clear_infinite_returns():
    scan = scan_fixture()
    scan.ranges = [float('inf')]*360
    route.check_rear_scan(scan, -0.032, 0, 0)


@pytest.mark.parametrize('fault', [None, 'scan_stale', 'odom_stale', 'stuck', 'interrupt', 'active'])
def test_departure_stops_on_all_exits(monkeypatch, fault):
    """Exercise the actual control loop, including stop publication and cleanup."""
    clock, commands, position = [0.0], [], [0.0]
    monkeypatch.setattr(route.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(route.time, 'sleep', lambda duration: clock.__setitem__(0, clock[0]+duration))
    monkeypatch.setattr(route.rclpy, 'ok', lambda: True)
    nav = SimpleNamespace()
    nav._verify_departure_pipeline = lambda: None
    nav.get_topic_names_and_types = lambda: [('navigate_to_pose/_action/status', ['action_msgs/msg/GoalStatusArray'])]
    callbacks = []
    nav.create_subscription = lambda msg_type, topic, callback, qos: callbacks.append(callback) or callback
    destroyed = []
    nav.destroy_subscription = lambda sub: destroyed.append(sub)
    nav.wait_until_stopped = lambda timeout: True
    nav._departure_pose = lambda: (position[0], 0.0, 0.0)
    nav._check_departure_scan = lambda: None
    from builtin_interfaces.msg import Time
    nav.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    nav.create_publisher = lambda *args: SimpleNamespace(publish=lambda msg: commands.append(msg))
    nav.destroy_publisher = lambda pub: destroyed.append(pub)

    def spin(node, timeout_sec):
        clock[0] += 0.05
        if commands:
            if fault in ('scan_stale', 'odom_stale'):
                raise RuntimeError(fault)
            if fault == 'interrupt':
                raise KeyboardInterrupt()
            if fault == 'active':
                callbacks[0](SimpleNamespace(status_list=[SimpleNamespace(status=2)]))
            if fault != 'stuck':
                position[0] += commands[-1].twist.linear.x * 0.1
    monkeypatch.setattr(route.rclpy, 'spin_once', spin)
    if fault:
        with pytest.raises(KeyboardInterrupt if fault == 'interrupt' else RuntimeError):
            route.WaypointNavigator.pre_backup(nav, 0.1, 0.05, 20)
    else:
        route.WaypointNavigator.pre_backup(nav, 0.1, 0.05, 20)
        assert 0.1 <= -position[0] < 0.11
    assert any(msg.twist.linear.x < 0 for msg in commands)
    assert all(-0.05 <= msg.twist.linear.x <= 0 and msg.twist.angular.z == 0 for msg in commands)
    assert all(msg.twist.linear.x == 0 for msg in commands[-10:])
    assert len(destroyed) == 2


@pytest.mark.parametrize('stale_header, stale_arrival', [(True, False), (False, True)])
def test_departure_sensor_freshness_checks_message_stamp_and_receipt(monkeypatch, stale_header, stale_arrival):
    monkeypatch.setattr(route.time, 'monotonic', lambda: 2.0)
    nav = SimpleNamespace(odom_message=route.Odometry(), scan_message=scan_fixture(),
                          odom_received_at=0.0 if stale_arrival else 2.0,
                          scan_received_at=0.0 if stale_arrival else 2.0,
                          _fresh_stamp=lambda *args: not stale_header)
    with pytest.raises(RuntimeError, match='odom'):
        route.WaypointNavigator._departure_pose(nav)
    with pytest.raises(RuntimeError, match='scan'):
        route.WaypointNavigator._check_departure_scan(nav)
