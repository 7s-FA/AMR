"""도킹 시험 YAML 로드. ROS/카메라를 시작하기 전에 검사합니다."""
import math
from pathlib import Path
import yaml

KEYS = ('marker_mm', 'gap_mm', 'target_cm', 'camera_forward_mm', 'camera_left_mm', 'align_min_m', 'align_max_m', 'align_lateral_m', 'align_yaw_deg', 'align_goal_m', 'settle_s', 'start_min_m', 'start_max_m', 'too_close_m', 'align_max_speed_mps', 'align_max_yaw_rate_rps', 'align_distance_gain', 'approach_slow_gain', 'approach_stop_m', 'straight_max_speed_mps', 'straight_distance_gain', 'straight_lateral_m', 'straight_yaw_deg', 'reverse_tolerance_m', 'overshoot_m', 'stop_remaining_m', 'progress_step_m', 'progress_timeout_s', 'stationary_speed_mps', 'stationary_yaw_rate_rps', 'stop_stable_s', 'stop_timeout_s', 'control_period_s', 'vision_timeout_s', 'odom_timeout_s', 'odom_stamp_timeout_s', 'camera_timeout_s', 'test_timeout_s')

KEYS += (
    "align_min_speed_mps",
    "straight_min_speed_mps",
    "prep_goal_m", "prep_min_m", "prep_max_m",
    "line_lookahead_m", "line_heading_gain", "line_turn_only_deg", "line_max_board_bearing_deg",
    "vision_reacquire_timeout_s", "vision_reacquire_stable_s",
    "alignment_check_timeout_s", "alignment_turn_timeout_s", "align_min_yaw_rate_rps",
)

def load_params(path, overrides):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != set(KEYS):
        found = set(data) if isinstance(data, dict) else set()
        raise ValueError(f"설정 키 확인: 누락={set(KEYS)-found}, 알 수 없는 키={found-set(KEYS)}")
    for key, value in overrides.items():
        if value is not None:
            data[key] = value
    for key, value in data.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{key}: 유효한 숫자를 입력하세요 (설치 위치는 실측 필요)")
        if key not in ("camera_forward_mm", "camera_left_mm"):
            if value < 0 or (value == 0 and key != "gap_mm"):
                raise ValueError(f"{key}: 양수여야 합니다 (gap_mm은 0 허용)")
    if not 1 <= data["target_cm"] <= 15:
        raise ValueError("target_cm: 1~15cm 범위로 설정하세요")
    if not (data["target_cm"] / 100 < data["too_close_m"] < data["align_min_m"] <= data["align_goal_m"] <= data["align_max_m"] <= data["start_max_m"]):
        raise ValueError("거리 관계 확인: target < too_close < align_min <= align_goal <= align_max <= start_max")
    if not data["too_close_m"] < data["start_min_m"] <= data["align_max_m"]:
        raise ValueError("start_min_m은 too_close_m보다 크고 align_max_m 이하여야 합니다")
    if not data["too_close_m"] <= data["approach_stop_m"] <= data["align_min_m"]:
        raise ValueError("approach_stop_m은 too_close_m~align_min_m 범위여야 합니다")
    if any(data[k] >= 90 for k in ("align_yaw_deg", "straight_yaw_deg", "line_turn_only_deg", "line_max_board_bearing_deg")):
        raise ValueError("각도 허용값은 90도 미만이어야 합니다")
    if data["stop_stable_s"] >= data["stop_timeout_s"]:
        raise ValueError("stop_stable_s는 stop_timeout_s보다 작아야 합니다")

    for phase in ("align", "straight"):
        minimum = data[f"{phase}_min_speed_mps"]
        maximum = data[f"{phase}_max_speed_mps"]

        if minimum < 0.01:
            raise ValueError(
                f"{phase}_min_speed_mps는 "
                "현재 드라이버의 전달 단위에 맞춰 0.01 이상이어야 합니다."
            )

        if minimum > maximum:
            raise ValueError(
                f"{phase}_min_speed_mps는 "
                f"{phase}_max_speed_mps 이하여야 합니다."
            )

    if not (data["align_max_m"] < data["prep_min_m"] <= data["prep_goal_m"] <= data["prep_max_m"] <= data["start_max_m"]):
        raise ValueError("거리 관계: align_max < prep_min <= prep_goal <= prep_max <= start_max")
    if data["vision_reacquire_stable_s"] >= data["vision_reacquire_timeout_s"]:
        raise ValueError("재검출 안정 시간은 재검출 제한시간보다 작아야 합니다")
    if data["settle_s"] >= data["alignment_check_timeout_s"]:
        raise ValueError("정렬 유지 시간은 정렬 확인 제한시간보다 작아야 합니다")
    if not 0.01 <= data["align_min_yaw_rate_rps"] <= data["align_max_yaw_rate_rps"]:
        raise ValueError("정렬 최소 각속도는 0.01 이상, 최대 각속도 이하여야 합니다")
    return data
