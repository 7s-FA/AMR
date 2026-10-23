#!/usr/bin/env python3
# ========================================================================
# 역할: 도킹 1회의 계산 과정을 표로 보기 (2026-10-13 추가). 읽기만 한다.
#       카메라 측정(거리·좌우·각도) → 제어기 판단(상태·이유) → 보낸 명령(v,w) → 바퀴 odom(v,w) 을 0.5초 간격으로 보여 준다.
# 실행: python3 dock_log.py [burger1|burger2] [로그폴더 이름 일부]   (기본: 가장 최근 도킹)
# ========================================================================
import bisect, glob, json, math, os, sys
from pathlib import Path

robot = sys.argv[1] if len(sys.argv) > 1 else ('burger1' if Path('/home/example_robot/AMR/.runtime/M1').exists() else 'burger2')
key = sys.argv[2] if len(sys.argv) > 2 else ''
M = 'M1' if robot == 'burger1' else 'M2'
L = f'/home/example_robot/AMR/.runtime/{M}/final_robot_ws/data/{robot}/docking_logs'
runs = [d for d in sorted(glob.glob(L + '/20*')) if os.path.exists(d + '/summary.json') and key in os.path.basename(d)]
if not runs:
    sys.exit('도킹 로그 없음')
d = runs[-1]
r = [json.loads(l) for l in open(d + '/events.jsonl')]
st = [x for x in r if x.get('kind') == 'status' and x.get('state')]
cmd = [x for x in r if x.get('kind') == 'command']; ct = [x['monotonic_s'] for x in cmd]
obs = [x for x in r if x.get('kind') == 'observation']; ot = [x['monotonic_s'] for x in obs]
summ = json.load(open(d + '/summary.json'))
cfg = {}
for line in open(d + '/00_docking.yaml'):
    s = line.split('#')[0].strip()
    for k in ('staging_distance_m', 'lateral_tolerance_m', 'yaw_tolerance_deg', 'angular_speed_scale', 'align_speed_mps', 'final_speed_mps'):
        if s.startswith(k + ':'): cfg.setdefault(k, s.split(':')[1].strip())
print(f'도킹 로그: {os.path.basename(d)}   모드: {st[0].get("docking_mode") if st else "?"}   결과: {summ.get("exit_reason")}')
print('설정: ' + ', '.join(f'{k}={v}' for k, v in cfg.items()) + '  (parking/assembly 는 각 섹션 값이 덮어씀)')
print('열: 시간 | 상태 | 거리z | 좌우by | yaw | 정면선거리 | 보낸명령 v·w | 바퀴odom v·w | 카메라 관측(유효/이유) | 제어기 판단 이유')
print('     좌우by = 보드 중심이 로봇 기준 왼쪽(+)/오른쪽(-) 몇 mm, yaw = 로봇과 보드 정면의 각도, 정면선거리 = 보드 정면선에서 벗어난 거리')
t0 = st[0]['monotonic_s'] if st else 0
nxt = -1.0
for x in st if len(st) < 3 else [y for y in r if y.get('kind') == 'status' and y.get('state')]:
    t = x['monotonic_s'] - t0
    if t < nxt and x is not st[-1]: continue
    nxt = t + 0.5
    g = lambda k, sc=1, f='%6.1f': (f % (x[k]*sc)) if isinstance(x.get(k), (int, float)) else '   -  '
    cross = '   -  '
    if all(isinstance(x.get(k), (int, float)) for k in ('lateral_error_m', 'yaw_error_deg', 'camera_distance_m')):
        th = math.radians(x['yaw_error_deg'])
        cross = '%+6.1f' % ((x['lateral_error_m']*math.cos(th) - x['camera_distance_m']*math.sin(th))*1000)
    i = bisect.bisect_right(ct, x['monotonic_s']) - 1
    c = cmd[i] if i >= 0 else {'linear_mps': 0, 'angular_rps': 0}
    j = bisect.bisect_right(ot, x['monotonic_s']) - 1
    o = obs[j] if j >= 0 else {}
    ov = '유효' if o.get('pose_valid') else (o.get('reason') or '-')[:22]
    print(f'{t:5.1f}s {x["state"][:13]:13s} z{g("camera_distance_m",100)}cm by{g("lateral_error_m",1000,"%+6.1f")}mm yaw{g("yaw_error_deg",1,"%+5.1f")}° '
          f'정면선{cross}mm | 명령 v{c["linear_mps"]*100:5.1f}cm/s w{math.degrees(c["angular_rps"]):+5.1f}°/s | '
          f'odom v{g("odom_linear_mps",100,"%5.1f")} w{g("odom_angular_rps",57.2958,"%+5.1f")} | {ov:22s} | {(x.get("reason") or "")[:34]}')
