# ========================================================================
# 역할: 정지 래치·속도 상한 파일(data/<로봇>/action_gate.json) 읽기 함수 모음.
#       관제 목표가 있을 때 모터 출력 = min(요청 속도, 최대속도×속도%), 래치·하트비트 끊김이면 0.
# 사용처: motion_owner.py(모든 모터 출력에 적용), backend.py(래치 쓰기/읽기), operation.py, run_selected_waypoints.sh·sequence_runner.py(출차 속도).
# ========================================================================
"""Robot-local stop latch and action speed ceiling at the sole motor publisher."""
import json, math, time
from pathlib import Path

# 래치 파일 읽기. 없으면 None(개별 시험), 깨졌으면 안전하게 '정지'로 본다.
def load_gate(path):
    try:
        value=json.loads(Path(path).read_text())
        if not isinstance(value,dict):raise ValueError('gate must be an object')
        return value
    except FileNotFoundError:return None
    except (ValueError,OSError):return {'estop':True,'reason':'invalid_action_gate'}

# 속도(v,w)에 래치를 적용. 정지 래치·하트비트 1초 초과·값 오류면 0, 아니면 속도% 상한으로 자른다.
def gate_output(path,v,w,now=None):
    now=time.monotonic() if now is None else now
    g=load_gate(path)
    if g is None:return v,w  # Existing individual/local operation, no action yet.
    if g.get('estop',True):return 0.,0.
    if not g.get('goal_id'):return v,w
    try:
        age=now-float(g['heartbeat']);percent=float(g['speed_percent'])
        if not (0<=age<=1.0 and math.isfinite(percent) and 0<percent<=100):return 0.,0.
        linear=float(g['max_linear_mps'])*percent/100
        angular=float(g['max_angular_rps'])*percent/100
        if not (0<linear<=.08 and 0<angular<=1.):return 0.,0.
        return max(-linear,min(linear,v)),max(-angular,min(angular,w))
    except (KeyError,TypeError,ValueError):return 0.,0.

# 출차(후진) 속도. 관제 목표가 있으면 속도% 를 적용, 정지 래치면 출차를 거부한다.
def departure_speed(path):
    g=load_gate(path)
    if not g or not g.get('goal_id'):return .045
    v,_=gate_output(path,.045,0.)
    if v<=0:raise RuntimeError('Action stop latch or stale heartbeat prevents departure')
    return v
