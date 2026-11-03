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


@pytest.mark.parametrize('robot', ['M1','M2'])
def test_named_approaches_keep_final_pose_and_warehouse_route(robot):
    base, stations = configs(robot)
    original = copy.deepcopy(stations)
    for v in original['variants'].values():
        v.pop('approach_curve',None);v.pop('approach_straight_m',None)
    for selection in ('mat', '1', '2', '3', '4', '12'):
        assert build_route(base, stations, selection) == build_route(base, original, selection)
    for selection in ('asm', 'park', 'rest'):
        old = build_route(base, original, selection)
        new = build_route(base, stations, selection)
        assert len(new['waypoints']) == 1
        assert new['waypoints'] == old['waypoints']
        assert new['arrival_tuning'] == old['arrival_tuning']
        assert new['waypoints'][0]['mode'] == 'forward'
        if selection == 'park':
            assert new['waypoints'][0]['yaw'] == 180
            assert stations['terminal']['park'] == 'park'


@pytest.mark.parametrize('value',[0,.06,float('nan'),float('inf'),True,'0.18',.41])
def test_invalid_straight_approach_rejected(value):
    base,stations=configs();stations['variants']['way4_rest']['approach_straight_m']=value
    with pytest.raises(ValueError):build_route(base,stations,'rest')

def test_conflicting_approach_types_rejected():
    base,stations=configs();stations['variants']['way3_asm'].update(approach_straight_m=.18,
        approach_curve=dict(radius_m=.18,straight_m=.12,entry_m=.18,side='left'))
    with pytest.raises(ValueError):build_route(base,stations,'asm')


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
    stations['variants']['way3_asm']['approach_curve']=dict(radius_m=.18,straight_m=.12,entry_m=.18,side='left')
    stations['variants']['way3_asm']['approach_curve'][key] = value
    with pytest.raises(ValueError):
        build_route(base, stations, 'asm')


def test_route_generation_does_not_mutate_canonical_input():
    base, stations = configs()
    before = copy.deepcopy((base, stations))
    build_route(base, stations, 'asm')
    assert (base, stations) == before
