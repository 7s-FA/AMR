"""Burger2 behavior ported to Burger1, without ROS nodes or motion."""
import math
import subprocess

import pytest
import yaml

from test_directional_waypoints import FakeNavigator, ROOT, route


def test_robot_has_four_forward_points_and_no_reverse_controller():
    points = yaml.safe_load((ROOT / 'config/waypoints_burger1.yaml').read_text())['waypoints']
    assert len(points) == 4
    assert all(point['mode'] == 'forward' for point in points)
    params = yaml.safe_load((ROOT / 'config/nav2_burger1_params.yaml').read_text())
    controller = params['controller_server']['ros__parameters']
    assert controller['controller_plugins'] == ['FollowPath', 'FollowPositionForward']
    assert 'precision_goal_checker' not in controller['goal_checker_plugins']
    assert 'waffle_navigation::PrecisionPose' not in controller['FollowPositionForward']['critics']
    assert 'FollowPositionReverse' not in controller
    assert params['amcl']['ros__parameters']['set_initial_pose']


@pytest.mark.parametrize('selection,count', [('1', 1), ('2', 1), ('3', 1), ('4', 1), ('12', 2)])
def test_numbered_shortcuts_validate_without_motion(selection, count):
    script = ROOT.parents[2] / 'robot/burger1/navigation/run_selected_waypoints.sh'
    if not script.is_file():
        pytest.skip('Robot workspace layout required; host shell branches have a separate hermetic regression suite')
    result = subprocess.run(['bash', str(script), selection, '--dry-run'],
                            capture_output=True, text=True, check=True)
    assert result.stdout.count('mode=forward') == count
    assert 'mode=reverse' not in result.stdout
    pending = ROOT.parents[2] / 'data/burger1/departure_pending'
    if pending.exists():
        assert '0.15 m' in result.stdout and '180°' in result.stdout
    else:
        assert '출차 동작 생략' in result.stdout
    assert '최종 방향 목표 허용오차: 3°' in result.stdout
    assert '각 좌표까지 Nav2 전진 피드백 주행 → 정지 → 해당 목표 yaw 회전' in result.stdout
    assert '5cm 이내 저속 XY/yaw 동시 보정' not in result.stdout
    assert '최종 진입점:' not in result.stdout
    assert '마지막 접근 전환:' not in result.stdout


def test_missed_target_axis_stops_lateral_miss_but_allows_short_segments():
    assert route.missed_target_axis((0, 0, 0), {'x': 1, 'y': 0}, (.9, .2, 0), .22)
    assert not route.missed_target_axis((0, 0, 0), {'x': 1, 'y': 0}, (.5, .2, 0), .54)
    assert not route.missed_target_axis((0, 0, 0), {'x': .2, 'y': 0}, (.19, .2, 0), .20)


def test_intermediate_handoff_requires_confirmed_stop():
    for stopped in (False, True):
        nav = FakeNavigator(stopped=stopped)
        nav.isTaskComplete = lambda: False
        nav.current_map_pose = lambda timeout: (.035, 0, 0)
        outcome = route.wait_for_task(nav, route.time.monotonic()+2, 1,
                                     waypoint={'x': 0, 'y': 0}, intermediate_handoff=True)
        assert outcome is (route.INTERMEDIATE_HANDOFF if stopped else False)


def test_explicit_final_yaw_uses_at_most_two_corrections():
    nav = FakeNavigator()
    nav.final_yaw_tolerance = math.radians(3)
    nav.map_poses = [(0, 0, math.radians(angle)) for angle in (12, 5, 4)]
    assert not route.finish_waypoint(nav, {'x': 0, 'y': 0, 'yaw': 0},
                                     route.time.monotonic()+30, 1)
    assert len(nav.spin_requests) == 2


def test_timed_departure_and_initial_spin_happen_once_for_two_points():
    nav = FakeNavigator()
    departures = []
    nav.pre_backup_timed = lambda distance, speed, deadline: departures.append((distance, speed))
    points = [dict(x=0, y=0, yaw=0, mode='forward'), dict(x=.3, y=0, yaw=0, mode='forward')]
    assert route.run_waypoints(nav, points, 30, ROOT/'behavior_trees',
                               pre_backup_distance=.15, pre_backup_speed=.05,
                               pre_backup_open_loop=True, pre_turn_angle_deg=180,
                               continuous_intermediate=True) == 0
    assert departures == [(.15, .05)]
    assert len(nav.spin_requests) == 1
    assert nav.spin_requests[0][0] == pytest.approx(math.pi)
    assert len(nav.goals) == 2
