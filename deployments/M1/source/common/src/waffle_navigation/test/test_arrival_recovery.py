from test_directional_waypoints import FakeNavigator, route, ROOT
import pytest
import yaml

@pytest.mark.parametrize('poses,expected,attempts', [
    ([(0.024,0,0),(0.01,0,0),(0.01,0,0)],0,2),
    ([(0,0,0),(0.024,0,0),(0.01,0,0),(0.01,0,0)],0,2),
    ([(0.04,0,0)]*3,1,3),
    ([(0.11,0,0)],1,1),
])
def test_bounded_position_recovery(poses,expected,attempts):
    n=FakeNavigator(results=[route.TaskResult.SUCCEEDED]*3)
    n.xy_tolerance=n.intermediate_xy_tolerance=0.02
    n.map_poses=poses
    p=[dict(x=0.,y=0.,yaw=0.,mode='reverse')]
    assert route.run_waypoints(n,p,30,ROOT/'behavior_trees')==expected
    assert len(n.goals)==attempts
    assert all(g[1].endswith('navigate_reverse.xml') for g in n.goals)

def test_goal_and_local_control_share_map_frame():
    cfg=yaml.safe_load((ROOT/'config/nav2_burger1_params.yaml').read_text())
    assert cfg['local_costmap']['local_costmap']['ros__parameters']['global_frame']=='burger1/map'

def test_recorded_143838_first_arrival_proceeds_without_reapproach():
    # Recorded pre/post-spin poses: 3.7 mm -> 28.4 mm; yaw 0.8 deg after Spin.
    cfg=yaml.safe_load((ROOT/'config/waypoints.yaml').read_text())
    n=FakeNavigator()
    n.xy_tolerance=cfg['arrival_tuning']['xy_goal_tolerance']
    n.intermediate_xy_tolerance=cfg['arrival_tuning']['intermediate_xy_tolerance']
    n.yaw_tolerance=cfg['arrival_tuning']['yaw_goal_tolerance']
    n.map_poses=[(.41845,.03307,3.24917),(.40811,.00991,1.57072)]
    assert route.run_waypoints(n,cfg['waypoints'][:2],30,ROOT/'behavior_trees')==0
    assert len(n.goals)==2
    assert len(n.spin_requests)==1
    assert n.goals[1][1].endswith('navigate_forward.xml')

def test_same_post_turn_error_is_rejected_at_final_destination():
    n=FakeNavigator();n.xy_tolerance=.02;n.intermediate_xy_tolerance=.04;n.yaw_tolerance=.25
    n.map_poses=[(.41845,.03307,3.24917),(.40811,.00991,1.57072)]
    p=dict(x=.41762385,y=.036671807,yaw=90.760873754,mode='reverse')
    assert not route.finish_waypoint(n,p,route.time.monotonic()+30,1,is_final=True)
    assert n.position_retry_requested


@pytest.mark.parametrize('after,expected', [(0.022, True), (0.04, True), (0.041, False)])
def test_burger2_recorded_final_spin_position_shift(after, expected):
    # Sep 30: reached 9 mm, spun 10.3 degrees, ended at 22 mm / -1.5 degrees.
    n = FakeNavigator()
    n.xy_tolerance = .02
    n.final_yaw_tolerance = route.math.radians(3)
    n.final_post_turn_xy_tolerance = .04
    n.map_poses = [(.009, 0, route.math.radians(-10.3)),
                   (after, 0, route.math.radians(1.5))]
    assert route.finish_waypoint(n, dict(x=0, y=0, yaw=0),
                                route.time.monotonic()+30, 1) is expected
    assert len(n.spin_requests) == 1


def test_post_turn_allowance_does_not_relax_initial_arrival():
    n = FakeNavigator()
    n.xy_tolerance = .02
    n.final_post_turn_xy_tolerance = .04
    n.map_poses = [(.022, 0, .2)]
    assert not route.finish_waypoint(n, dict(x=0, y=0, yaw=0),
                                     route.time.monotonic()+30, 1)
    assert not n.spin_requests


def test_post_turn_allowance_preserves_yaw_gate():
    n = FakeNavigator()
    n.xy_tolerance = .02
    n.final_yaw_tolerance = route.math.radians(3)
    n.final_post_turn_xy_tolerance = .04
    n.map_poses = [(.009, 0, .2), (.022, 0, .1), (.022, 0, .1)]
    assert not route.finish_waypoint(n, dict(x=0, y=0, yaw=0),
                                     route.time.monotonic()+30, 1)
