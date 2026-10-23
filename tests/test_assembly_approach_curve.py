"""Assembly route geometry and isolation; no ROS or robot commands."""
import copy
import math
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'common/navigation'))
from station_routes import build_route, curved_entry


def configs(robot='M1'):
    return tuple(yaml.safe_load((ROOT / robot / 'config' / name).read_text())
                 for name in ('waypoints.yaml', 'routes.yaml'))


def test_only_m1_assembly_expands_and_keeps_final_pose_and_tuning():
    base, stations = configs()
    original = copy.deepcopy(stations)
    del original['variants']['way3_asm']['approach_curve']
    for selection in ('mat', 'park', 'rest', '1', '2', '3', '4', '12'):
        assert build_route(base, stations, selection) == build_route(base, original, selection)
    for selection in ('asm', 'way3_asm'):
        old = build_route(base, original, selection)
        new = build_route(base, stations, selection)
        assert len(new['waypoints']) == 5
        assert new['waypoints'][-1] == old['waypoints'][-1]
        assert new['arrival_tuning'] == old['arrival_tuning']
        assert all(p['mode'] == 'forward' for p in new['waypoints'])
        # Last segment points exactly toward assembly (+X), with 12 cm run-in.
        start, end = new['waypoints'][-2:]
        assert end['x'] - start['x'] == pytest.approx(.12)
        assert start['y'] == end['y'] and start['yaw'] == end['yaw'] == 0
        # All vias are separated beyond RemovePassedGoals' 6 cm radius.
        assert all(math.hypot(a['x']-b['x'], a['y']-b['y']) > .12
                   for a, b in zip(new['waypoints'][:-2], new['waypoints'][1:-1]))
    assert 'approach_curve' not in configs('M2')[1]['variants']['way3_asm']


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_curve_rotates_with_target_heading_and_mirrors(side, sign):
    cfg = dict(radius_m=.18, straight_m=.12, entry_m=.18, side=side)
    base = curved_entry(dict(x=0., y=0., yaw=0.), cfg)
    rotated = curved_entry(dict(x=2., y=-3., yaw=90.), cfg)
    assert base[0]['y'] == pytest.approx(sign * .36)
    for p, q in zip(base, rotated):
        assert q['x'] == pytest.approx(2 - p['y'])
        assert q['y'] == pytest.approx(-3 + p['x'])
        assert q['yaw'] == pytest.approx((p['yaw'] + 90) % 360)


@pytest.mark.parametrize('key,value', [
    ('radius_m', float('nan')), ('radius_m', float('inf')),
    ('radius_m', True), ('radius_m', '0.18'),
    ('straight_m', 0), ('entry_m', .41), ('side', 'invalid'),
])
def test_invalid_curve_rejected_before_route_is_returned(key, value):
    base, stations = configs()
    stations['variants']['way3_asm']['approach_curve'][key] = value
    with pytest.raises(ValueError):
        build_route(base, stations, 'asm')


def test_route_generation_does_not_mutate_canonical_input():
    base, stations = configs()
    before = copy.deepcopy((base, stations))
    build_route(base, stations, 'asm')
    assert (base, stations) == before
