"""Regression for curved Nav2 routes, stale plans, and sustained path departures."""
from types import SimpleNamespace

import pytest

from test_directional_waypoints import route


def test_distance_uses_curved_polyline_instead_of_start_goal_axis():
    points = [(0., 0.), (1., 0.), (1., 1.)]
    assert route.planned_path_distance(points, (1., .6, 0.)) == pytest.approx(0.)
    assert route.planned_path_distance(points, (.9, .8, 0.)) == pytest.approx(.1)
    assert route.planned_path_distance(points, (2., 2., 0.)) == pytest.approx(2**.5)
    assert route.planned_path_distance([], (0., 0., 0.)) is None


def test_brief_deviation_and_missing_plan_do_not_cancel():
    guard = route.PlannedPathGuard()
    assert not guard.update(.2, 1.)
    assert not guard.update(.2, 1.9)
    assert not guard.update(None, 2.)
    assert not guard.update(.2, 2.1)
    assert not guard.update(.1, 2.2)
    assert not guard.update(.2, 3.)
    assert guard.update(.2, 4.)


@pytest.mark.parametrize('kind', ['old_goal', 'old_receipt', 'old_stamp', 'wrong_endpoint', 'empty'])
def test_unrelated_or_stale_plan_is_not_used(monkeypatch, kind):
    monkeypatch.setattr(route.time, 'monotonic', lambda: 10.)
    nav = SimpleNamespace(plan_message=SimpleNamespace(header=SimpleNamespace(stamp=object())),
                          plan_received_at=9., plan_points=((0., 0.), (1., 0.)),
                          _fresh_stamp=lambda stamp, timeout: kind != 'old_stamp')
    if kind == 'old_goal':
        nav.plan_received_at = 7.9
    if kind == 'old_receipt':
        nav.plan_received_at = 7.
    if kind == 'wrong_endpoint':
        nav.plan_points = ((0., 0.), (2., 0.))
    if kind == 'empty':
        nav.plan_points = ()
    assert route.WaypointNavigator.current_plan_offset(nav, {'x': 1., 'y': 0.}, (.5, .1, 0.), 8.) is None


def test_fresh_matching_plan_uses_segment_distance(monkeypatch):
    monkeypatch.setattr(route.time, 'monotonic', lambda: 10.)
    nav = SimpleNamespace(plan_message=SimpleNamespace(header=SimpleNamespace(stamp=object())),
                          plan_received_at=9., plan_points=((0., 0.), (1., 0.)),
                          _fresh_stamp=lambda stamp, timeout: True)
    assert route.WaypointNavigator.current_plan_offset(nav, {'x': 1., 'y': 0.}, (.5, .1, 0.), 8.) == pytest.approx(.1)
