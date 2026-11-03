"""Bounded pre-insertion marker recovery. Pure policy, no actuator output."""
import math

PREINSERT = ('PREALIGN', 'PRETURN', 'PRECHECK', 'VISION_APPROACH', 'FINALTURN', 'FINALCHECK')
MAX_ATTEMPTS = 2
MAX_DISTANCE = .15
MAX_SPEED = .03
# Full remaining withdrawal corridor plus conservative clearance, not a braking model.
CLEARANCE = .10
REAR = .204  # measured lidar-to-rear .14 + base-to-lidar .064
HALF_WIDTH = .135


def eligible(stage, reason, previous, previous_time, previous_odom, odom, now, count, too_close):
    if stage not in PREINSERT or count >= MAX_ATTEMPTS:
        return False
    if reason != 'reprojection_error' and not reason.startswith('missing_id_'):
        return False
    if previous is None or previous_odom is None or odom is None or not 0 <= now-previous_time <= 3:
        return False
    values = (*previous_odom, *odom, previous.get('plane_distance_m', math.nan))
    if not all(math.isfinite(v) for v in values):
        return False
    displacement = math.hypot(odom[0]-previous_odom[0], odom[1]-previous_odom[1])
    yaw = math.atan2(math.sin(odom[2]-previous_odom[2]), math.cos(odom[2]-previous_odom[2]))
    return (displacement <= .03 and abs(yaw) <= math.radians(5)
            and previous['plane_distance_m']-displacement > too_close)


def rear_clear(scan, now):
    """Fresh transformed rays must cover the rear half-plane without unknown gaps."""
    if not scan or not 0 <= now-scan['stamp'] <= .3:
        return False, '후방 라이다 수신 지연'
    angles=[]
    for a,x,y in scan['rays']:
        a=a % (2*math.pi)
        if not math.pi/2 <= a <= 3*math.pi/2:
            continue
        angles.append(a)
        if x is None or y is None or not math.isfinite(x) or not math.isfinite(y):
            return False, '후방 라이다 미확인 영역'
        if -REAR-MAX_DISTANCE-CLEARANCE <= x <= CLEARANCE and abs(y) <= HALF_WIDTH+CLEARANCE:
            return False, '후방 후진 공간에 장애물'
    angles.sort()
    edges=[math.pi/2,*angles,3*math.pi/2]
    if len(angles)<30 or max(b-a for a,b in zip(edges,edges[1:])) > math.radians(3):
        return False, '후방 라이다 감시 범위 부족'
    return True, ''
