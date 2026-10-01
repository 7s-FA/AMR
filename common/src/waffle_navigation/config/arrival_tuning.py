"""Validate optional waypoint arrival settings and merge without altering source YAML."""
import math
from copy import deepcopy
import yaml
KEYS = {"xy_goal_tolerance", "intermediate_xy_tolerance", "yaw_goal_tolerance",
        "max_rotational_vel", "min_rotational_vel", "rotational_acc_lim"}

def validate_collision_frames(params):
    """Behaviors pass unframed poses directly to the corresponding costmap."""
    behavior = params['behavior_server']['ros__parameters']
    for scope in ('local', 'global'):
        costmap_name = f'{scope}_costmap'
        costmap_frame = params[costmap_name][costmap_name]['ros__parameters']['global_frame']
        behavior_frame = behavior[f'{scope}_frame']
        if not costmap_frame or behavior_frame != costmap_frame:
            raise ValueError(
                f'충돌검사 좌표계 불일치: behavior_server.{scope}_frame='
                f'{behavior_frame}, {costmap_name}.global_frame={costmap_frame}. '
                '두 좌표계를 같게 설정하세요.')

def load_tuning(path):
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("웨이포인트 YAML은 매핑이어야 합니다.")
    tuning = data.get("arrival_tuning", {})
    if not isinstance(tuning, dict) or set(tuning) - KEYS:
        raise ValueError("arrival_tuning에 알 수 없는 항목이 있습니다.")
    if tuning and set(tuning) != KEYS:
        raise ValueError("arrival_tuning은 설명된 6개 항목을 모두 지정하세요.")
    for key, value in tuning.items():
        if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key}: 0보다 큰 유한한 숫자가 필요합니다.")
    if tuning:
        if tuning["min_rotational_vel"] > tuning["max_rotational_vel"]:
            raise ValueError("최소 회전속도는 최대 회전속도 이하여야 합니다.")
        if tuning["intermediate_xy_tolerance"] < tuning["xy_goal_tolerance"]:
            raise ValueError("정지 후 공차가 이동 공차보다 작으면 조기 중단될 수 있습니다. 같거나 크게 설정하세요.")
    return {k:float(v) for k,v in tuning.items()}
def merge_params(params, tuning):
    result = deepcopy(params)
    if not tuning:
        return result
    c = result["controller_server"]["ros__parameters"]
    c["position_goal_checker"]["xy_goal_tolerance"] = tuning["xy_goal_tolerance"]
    c["goal_checker"]["yaw_goal_tolerance"] = tuning["yaw_goal_tolerance"]
    for name in ("FollowPositionForward", "FollowPositionReverse"):
        if name in c:
            c[name]["xy_goal_tolerance"] = tuning["xy_goal_tolerance"]
    b = result["behavior_server"]["ros__parameters"]
    for key in ("max_rotational_vel", "min_rotational_vel", "rotational_acc_lim"):
        b[key] = tuning[key]
    return result
