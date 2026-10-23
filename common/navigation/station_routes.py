# ========================================================================
# 역할: 목적지 이름(mat/asm/rest/park 또는 번호)을 실제 웨이포인트 목록으로 바꿔 임시 경로 파일을 만든다.
#       좌표는 waypoints.yaml(공통 좌표), 목적지별 순서·방향·종단 동작은 routes.yaml(station_routes.yaml).
# 실행: run_selected_waypoints.sh 가 python3 station_routes.py <좌표> <routes> <목적지> <출력파일> 로 호출.
# 사용처: sequence_runner.py 가 terminal_mode() 로 종단 동작(dock/park/rest)을 정한다.
# ========================================================================
"""Named headings share canonical coordinates; build destination-specific routes."""
import argparse,copy,math
from pathlib import Path
import yaml

# 목적지의 종단 동작(dock/park/rest)을 돌려준다.
def terminal_mode(stations, destination):
    if destination not in stations.get('routes', {}):
        raise ValueError('Unknown destination: '+destination)
    mode = stations.get('terminal', {}).get(destination)
    if mode not in ('dock', 'park', 'rest'):
        raise ValueError('Missing or invalid terminal action: '+destination)
    return mode

def curved_entry(point, settings):
    """Guide a forward route onto the final heading before its unchanged endpoint.

    These are Nav2 via points, not velocity commands. The controller still
    checks obstacles and the route runner still verifies final XY and yaw.
    """
    if not isinstance(settings, dict) or set(settings) != {'radius_m', 'straight_m', 'entry_m', 'side'}:
        raise ValueError('approach_curve requires radius_m, straight_m, entry_m and side')
    radius, straight, entry = (settings[k] for k in ('radius_m', 'straight_m', 'entry_m'))
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           or not .12 <= v <= .4 for v in (radius, straight, entry)):
        raise ValueError('approach_curve lengths must be finite, within 0.12..0.4 m')
    if settings['side'] not in ('left', 'right'):
        raise ValueError('approach_curve side must be left or right')
    sign = 1 if settings['side'] == 'left' else -1
    yaw = math.radians(point['yaw']); c, s = math.cos(yaw), math.sin(yaw)
    offsets = [(-straight-radius, sign*(radius+entry)),
               (-straight-radius, sign*radius),
               (-straight-radius/math.sqrt(2), sign*radius*(1-1/math.sqrt(2))),
               (-straight, 0.)]
    result = [dict(x=point['x']+along*c-left*s, y=point['y']+along*s+left*c,
                   yaw=point['yaw'], mode='forward') for along, left in offsets]
    for index, via in enumerate(result):
        following = (result + [point])[index+1]
        via['yaw'] = math.degrees(math.atan2(following['y']-via['y'], following['x']-via['x'])) % 360
    return result

# 목적지 → 웨이포인트 이름 목록 → 좌표·방향·주행모드가 들어간 경로 사전.
def build_route(base, stations, selection):
    if selection in stations['routes']:
        names=stations['routes'][selection]
    elif selection in stations['variants']:
        names=[selection]
    elif selection in ['1','2','3','4','12']:
        return {'arrival_tuning':copy.deepcopy(base.get('arrival_tuning',{})),
                'waypoints':[dict(base['waypoints'][int(i)-1],mode='forward') for i in ([1,2] if selection=='12' else [int(selection)])]}
    else:raise ValueError('알 수 없는 경로: '+selection)
    result=[]
    for name in names:
        variant=stations['variants'][name];point=copy.deepcopy(base['waypoints'][variant['waypoint']-1]);point['mode']='forward'
        if 'face_waypoint' in variant:
            next_point=base['waypoints'][variant['face_waypoint']-1]
            if math.hypot(next_point['x']-point['x'],next_point['y']-point['y']) < .001:raise ValueError('방향 계산 대상 좌표가 같습니다.')
            point['yaw']=math.degrees(math.atan2(next_point['y']-point['y'],next_point['x']-point['x']))%360
        else:point['yaw']=float(variant['yaw_deg'])%360
        if not all(math.isfinite(float(point[k])) for k in ('x','y','yaw')):raise ValueError('좌표/방향이 유효하지 않습니다.')
        if 'approach_curve' in variant:
            result.extend(curved_entry(point, variant['approach_curve']))
        result.append(point)
    return {'arrival_tuning':copy.deepcopy(base.get('arrival_tuning',{})),'waypoints':result}

# 경로 파일을 쓰고 각 점을 출력.
def main():
    p=argparse.ArgumentParser();p.add_argument('base');p.add_argument('stations');p.add_argument('selection');p.add_argument('output');a=p.parse_args()
    route=build_route(yaml.safe_load(Path(a.base).read_text()),yaml.safe_load(Path(a.stations).read_text()),a.selection)
    Path(a.output).write_text(yaml.safe_dump(route,sort_keys=False))
    for i,point in enumerate(route['waypoints'],1):print(f'{a.selection} {i}: {point}',flush=True)
if __name__=='__main__':main()
