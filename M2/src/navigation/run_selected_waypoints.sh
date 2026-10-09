#!/usr/bin/env bash
# ========================================================================
# 역할: 목적지 이름(mat/asm/rest/park 등)으로 경로 파일을 만들고 웨이포인트 주행 실행.
#       출차 표시(departure_pending)가 있으면 엔코더 후진+180도 회전 인자를 붙이고, 관제 명령이면 재시도 인자를 붙인다.
#       시작 전 이동 잠금·준비(warm.sh)·카메라·모드 nav, 끝나면 모드 idle. 로그: data/<로봇>/motion_diagnostics/.
# 호출: sequence_runner.py (경로 단계). --dry-run 은 경로만 출력.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Robot-side numbered routes; --dry-run only prints the selected route.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$HERE/../../.." && pwd)"
ROBOT=$(basename "$(dirname "$HERE")")
SELECTION="${1:-}"
[[ -n "$SELECTION" ]] || { echo '경로 이름을 입력하세요.' >&2;exit 2; }
shift
TEMP="$(mktemp /tmp/burger2-route.XXXXXX.yaml)"
cleanup() { rm -f -- "$TEMP"; }
trap cleanup EXIT
python3 "$HERE/station_routes.py" "$ROOT/host_ws/src/waffle_navigation/config/waypoints_burger2.yaml" "$HERE/station_routes.yaml" "$SELECTION" "$TEMP"
DEPARTURE_FLAG="$ROOT/data/burger2/departure_pending"
WAYPOINT_ARGS=(--waypoints "$TEMP" --pre-backup-distance 0 --pre-turn-angle-deg 0 --final-yaw-tolerance-deg 3
      --final-post-turn-xy-tolerance 0.06 --position-arrival-retries 1
      --align-large-heading-before-navigation --prealign-heading-deg 100 --skip-intermediate-yaw)
# Host routes retry a failed coordinate without repeating departure or completed legs.
if [[ "${BURGER_COMMON_RETRY:-}" == 1 ]];then
  # The mission policy owns the retry budget; do not multiply inner retries.
  WAYPOINT_ARGS+=(--navigation-retries 0 --position-arrival-retries 0
                  --route-progress "$BURGER_ROUTE_PROGRESS")
  # Recovery alone retains stop-and-align; successful first attempts steer
  # while moving. A checkpoint exists only after the previous route started.
  if [[ -f "$BURGER_ROUTE_PROGRESS" ]];then
    WAYPOINT_ARGS+=(--align-before-navigation)
  fi
elif [[ "${BURGER_REQUEST_SOURCE:-}" == host ]] || python3 - "$ROOT/data/burger2/action_gate.json" <<'GATE'
import json,sys
try:
    gate=json.load(open(sys.argv[1]))
    sys.exit(0 if gate.get('goal_id') and not gate.get('estop', True) else 1)
except (OSError,ValueError,TypeError):
    sys.exit(1)
GATE
then
  WAYPOINT_ARGS+=(--navigation-retries 3)
fi
add_departure_args() {
  if [[ -f "$DEPARTURE_FLAG" ]]; then
    local flag_text
    flag_text=$(< "$DEPARTURE_FLAG")
    if [[ $flag_text == terminal_started_* ]]; then
      echo '중단된 도킹 상태: 경로 출차 금지, 동일 목적지 도킹 재시작 필요' >&2
      return 1
    fi
    DEPARTURE_VALUES=$(PYTHONPATH="$HERE" python3 - "$ROOT/data/$ROBOT/action_gate.json" <<'SPEED'
import sys
from action_gate import departure_speed
speed = departure_speed(sys.argv[1])
print(speed, speed * 3.5)
SPEED
)
    read -r DEPARTURE_SPEED DEPARTURE_DISTANCE <<< "$DEPARTURE_VALUES"
    # 이전 출차가 중간에 실패했으면 남은 동작만 다시 한다. 출차가 끝나면 경로 코드가 표시를 치운다.
    if [[ $flag_text == departure_backup_done* ]]; then
      DEPARTURE_DISTANCE=0
      echo '이전 출차에서 후진은 완료: 180도 회전부터 다시 실행'
    elif [[ $flag_text == departure_backup_partial* ]]; then
      local remaining
      remaining=$(awk '{print $2}' <<< "$flag_text")
      if [[ $remaining =~ ^[0-9]+(\.[0-9]+)?$ ]] && awk -v r="$remaining" -v d="$DEPARTURE_DISTANCE" 'BEGIN{exit !(r > 0 && r <= d)}'; then
        DEPARTURE_DISTANCE=$remaining
        echo "이전 출차 후진이 덜 됨: 남은 ${remaining} m 후진·180도 회전 실행"
      else
        echo '이전 출차 기록 확인 불가: 전체 출차(후진·180도 회전) 실행'
      fi
    else
      echo '도킹/파킹/rest 후 첫 주행: 엔코더 후진·180도 회전 실행'
    fi
    WAYPOINT_ARGS+=(--pre-backup-distance "$DEPARTURE_DISTANCE" --pre-backup-speed "$DEPARTURE_SPEED" --pre-backup-open-loop
           --pre-turn-angle-deg 180
           --departure-flag "$DEPARTURE_FLAG" --departure-consumed "$LOG_DIR/departure_consumed")
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
source "$HERE/mission_guard.sh";mission_guard burger2
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/burger2-motion-$UID.lock"
flock -n 9 || { echo 'Burger2 주행/도킹 명령이 이미 실행 중입니다.' >&2; exit 1; }
if systemctl --user is-active --quiet burger2-rest.service || systemctl --user is-active --quiet burger2-docking.service || pgrep -u "$USER" -f '[d]ocking_node.py|[/]nav2_waypoints' >/dev/null; then
  echo '기존 주행/도킹을 먼저 종료하세요.' >&2; exit 1
fi
# Do not pass the caller's action/waypoint arguments into preparation.
source "$HERE/warm.sh" ""
if [[ "$SELECTION" == asm || "$SELECTION" == mat ]];then bash "$HERE/ensure_docking_ready.sh";fi
bash "$HERE/ensure_camera.sh"
bash "$HERE/set_mode.sh" nav
systemctl --user is-active --quiet burger2-nav2.service || {
  echo 'Nav2 주행 서비스가 준비되지 않았습니다. b1_ready 및 서비스 로그를 확인하세요.' >&2; exit 1;
}
# Camera stays warm; never commands motors.
LOG_DIR="$ROOT/data/burger2/motion_diagnostics/way${SELECTION}_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$LOG_DIR"
RECORDER_PID=
cleanup_route() {
  status=$?
  trap - EXIT
  if (( status != 0 )); then
    echo '주행 실패/중단: Burger2 모터 명령을 중단하고 위치 추정은 유지합니다.' >&2
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
  /burger2/odom /burger2/amcl_pose /burger2/tf /burger2/tf_static \
  /burger2/cmd_vel_nav /burger2/cmd_vel_smoothed /burger2/cmd_vel \
  /burger2/plan /burger2/local_plan /burger2/collision_monitor_state \
  > "$LOG_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
sleep .2
if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
  cat "$LOG_DIR/recorder.log" >&2
  exit 1
fi
fi
# 출차 표시는 미리 치우지 않는다. 경로 코드가 후진 완료/부분 진행을 기록하고,
# 후진·회전이 모두 끝난 순간에만 치운다. 실패하면 다음 명령이 남은 출차부터 한다.
add_departure_args
bash "$HERE/run_waypoints.sh" "${WAYPOINT_ARGS[@]}" "$@" 2>&1 | tee "$LOG_DIR/waypoints.log"
