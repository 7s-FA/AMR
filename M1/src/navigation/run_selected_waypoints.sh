#!/usr/bin/env bash
# Robot-side numbered routes; --dry-run only prints the selected route.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$HERE/../../.." && pwd)"
ROBOT=$(basename "$(dirname "$HERE")")
SELECTION="${1:-}"
[[ -n "$SELECTION" ]] || { echo '경로 이름을 입력하세요.' >&2;exit 2; }
shift
TEMP="$(mktemp /tmp/burger1-route.XXXXXX.yaml)"
cleanup() { rm -f -- "$TEMP"; }
trap cleanup EXIT
python3 "$HERE/station_routes.py" "$ROOT/host_ws/src/waffle_navigation/config/waypoints_burger1.yaml" "$HERE/station_routes.yaml" "$SELECTION" "$TEMP"
DEPARTURE_FLAG="$ROOT/data/burger1/departure_pending"
WAYPOINT_ARGS=(--waypoints "$TEMP" --pre-backup-distance 0 --pre-turn-angle-deg 0 --final-yaw-tolerance-deg 3
      --final-post-turn-xy-tolerance 0.04 --position-arrival-retries 1 --align-large-heading-before-navigation --prealign-heading-deg 100 --skip-intermediate-yaw)
add_departure_args() {
  if [[ -f "$DEPARTURE_FLAG" ]]; then
    DEPARTURE_SPEED=$(PYTHONPATH="$HERE" python3 - "$ROOT/data/$ROBOT/action_gate.json" <<'SPEED'
import sys
from action_gate import departure_speed
print(departure_speed(sys.argv[1]))
SPEED
)
    WAYPOINT_ARGS+=(--pre-backup-distance 0.15 --pre-backup-speed "$DEPARTURE_SPEED"
           --pre-backup-open-loop --pre-turn-angle-deg 180)
    echo '도킹/파킹/rest 후 첫 주행: 출차 후진·180도 회전 실행'
  else
    echo '출차 동작 생략: 바로 웨이포인트 주행'
  fi
}
for argument in "$@"; do
  if [[ "$argument" == --dry-run ]]; then
    add_departure_args
    bash "$HERE/run_waypoints.sh" "${WAYPOINT_ARGS[@]}" "$@"
    exit
  fi
done
source "$HERE/mission_guard.sh";mission_guard burger1
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/burger1-motion-$UID.lock"
flock -n 9 || { echo 'Burger1 주행/도킹 명령이 이미 실행 중입니다.' >&2; exit 1; }
if systemctl --user is-active --quiet burger1-rest.service || systemctl --user is-active --quiet burger1-docking.service || pgrep -u "$USER" -f '[d]ocking_node.py|[/]nav2_waypoints' >/dev/null; then
  echo '기존 주행/도킹을 먼저 종료하세요.' >&2; exit 1
fi
# Do not pass the caller's action/waypoint arguments into preparation.
source "$HERE/warm.sh" ""
if [[ "$SELECTION" == asm || "$SELECTION" == mat ]];then bash "$HERE/ensure_docking_ready.sh";fi
bash "$HERE/ensure_camera.sh"
bash "$HERE/set_mode.sh" nav
systemctl --user is-active --quiet burger1-nav2.service || {
  echo 'Nav2 주행 서비스가 준비되지 않았습니다. b1_ready 및 서비스 로그를 확인하세요.' >&2; exit 1;
}
# Camera stays warm; never commands motors.
LOG_DIR="$ROOT/data/burger1/motion_diagnostics/way${SELECTION}_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$LOG_DIR"
RECORDER_PID=
cleanup_route() {
  status=$?
  trap - EXIT
  if (( status != 0 )); then
    echo '주행 실패/중단: Burger1 모터 명령을 중단하고 위치 추정은 유지합니다.' >&2
  fi
  if [[ -n "$RECORDER_PID" ]]; then
    kill -TERM "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" || true
  fi
  bash "$HERE/set_mode.sh" idle || true
  rm -f -- "$TEMP"
  exit "$status"
}
trap cleanup_route EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "주행 로그: $LOG_DIR/waypoints.log"
source "$HERE/nav_env.bash"
# Keep motion feedback and commanded velocities for distinguishing steering,
# localization shifts and wheel response. No camera/costmap images are recorded.
if [[ "$(printenv BURGER_RECORD_BAG || echo 0)" == 1 ]];then
ros2 bag record -o "$LOG_DIR/bag" \
  /burger1/odom /burger1/amcl_pose /burger1/tf /burger1/tf_static \
  /burger1/cmd_vel_nav /burger1/cmd_vel_smoothed /burger1/cmd_vel \
  /burger1/plan /burger1/local_plan /burger1/collision_monitor_state \
  > "$LOG_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
sleep .2
if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
  cat "$LOG_DIR/recorder.log" >&2
  exit 1
fi
fi
# Claim the one-shot departure only after readiness/recorder checks. Never
# automatically repeat it after a failed or interrupted departure.
add_departure_args
if [[ -f "$DEPARTURE_FLAG" ]]; then
  mv -- "$DEPARTURE_FLAG" "$LOG_DIR/departure_consumed"
fi
bash "$HERE/run_waypoints.sh" "${WAYPOINT_ARGS[@]}" "$@" 2>&1 | tee "$LOG_DIR/waypoints.log"
