from test_directional_waypoints import FakeNavigator, ROOT, route


POINTS = [
    {'x': 0.42062385, 'y': 0.036671807, 'yaw': 90.760873754, 'mode': 'forward'},
    {'x': 0.43906513, 'y': 0.360654172, 'yaw': 90.760873754, 'mode': 'forward'},
]


def test_way12_continues_without_intermediate_spin():
    nav = FakeNavigator()
    assert route.run_waypoints(nav, POINTS, 300, ROOT / 'behavior_trees',
                               continuous_intermediate=True) == 0
    assert len(nav.goals) == 2
    assert nav.spin_requests == []


def test_way12_aborts_when_second_leg_alignment_fails():
    nav = FakeNavigator()
    nav.align_for_travel = lambda waypoint, deadline, index: index == 1
    assert route.run_waypoints(nav, POINTS, 300, ROOT / 'behavior_trees',
                               continuous_intermediate=True) == 1
    assert len(nav.goals) == 1
    assert nav.spin_requests == []
