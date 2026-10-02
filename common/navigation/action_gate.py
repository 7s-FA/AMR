"""Robot-local stop latch and action speed ceiling at the sole motor publisher."""
import json, math, time
from pathlib import Path

def load_gate(path):
    try:
        value=json.loads(Path(path).read_text())
        if not isinstance(value,dict):raise ValueError('gate must be an object')
        return value
    except FileNotFoundError:return None
    except (ValueError,OSError):return {'estop':True,'reason':'invalid_action_gate'}

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

def departure_speed(path):
    g=load_gate(path)
    if not g or not g.get('goal_id'):return .045
    v,_=gate_output(path,.045,0.)
    if v<=0:raise RuntimeError('Action stop latch or stale heartbeat prevents departure')
    return v
