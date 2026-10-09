#!/usr/bin/env bash
# ========================================================================
# 역할: 로봇 운용 명령 모음. ready(전체 준비) / dock·park(도킹 시작) / stop·dock-stop(이동 중단, 모터 idle) /
#       off(서비스 전체 정지) / status(서비스 상태).
# 호출: operation.py(stop/ready), sequence_runner.py(dock/park, stop), 로봇 터미널.
# dock/park: 이동 잠금 → 본체·카메라·도킹 대기 준비 → systemd 로 terminal_wrapper.sh + start_docking.sh 실행 → 시작 확인.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
CAMERA_ROOT=${AMR_CAMERA}
[[ "$ROBOT" != burger1 ]] || CAMERA_ROOT=${AMR_CAMERA}
ACTION=$1
case "$ACTION" in
 status) exec systemctl --user status "$ROBOT-base.service" "$ROBOT-localization.service" "$ROBOT-nav2.service" "$ROBOT-motion-owner.service" "$ROBOT-nav-control.service" "$ROBOT-docking.service" "$ROBOT-rest.service" --no-pager ;;
 stop|off|dock-stop)
  bash "$HERE/set_mode.sh" idle || true
  # Prepared waypoint commands must receive the same interruption as cold CLI commands.
  systemctl --user kill --kill-whom=main --signal=USR1 burger1-waypoint-ready.service 2>/dev/null || true
  systemctl --user kill --kill-whom=main --signal=USR1 burger1-rest-ready.service 2>/dev/null || true
  pkill -INT -u "$USER" -f '[/]nav2_waypoints' || true
  for unit in "$ROBOT-docking.service" "$ROBOT-rest.service"; do
    state=$(systemctl --user show "$unit" --property=LoadState --value 2>/dev/null || true)
    [[ "$state" == loaded ]] || continue
    systemctl --user stop "$unit"
  done
  if [[ "$ACTION" == off ]];then systemctl --user stop "$ROBOT-nav2.service" "$ROBOT-localization.service" "$ROBOT-motion-owner.service" "$ROBOT-nav-control.service" "$ROBOT-camera.service" "$ROBOT-docking-ready.service" "$ROBOT-base.service";fi
  exit ;;
 ready|dock|park) ;;
 *) echo '사용법: manage.sh ready|dock|park|dock-stop|stop|off|status';exit 2 ;;
esac
source "$HERE/mission_guard.sh";mission_guard "$ROBOT"
RUNTIME=$(printenv XDG_RUNTIME_DIR || echo /tmp)
if [[ "$ACTION" == ready ]]; then
 # Preparation blocks new missions, but is not itself motor movement.
 exec 8>"$RUNTIME/$ROBOT-prepare-$UID.lock"
 flock -n 8 || { echo '다른 준비 명령 실행 중입니다.' >&2;exit 1; }
 python3 "$HERE/ready_parallel.py"
 echo "$ROBOT 준비 완료. 위치 추정 유지."
 exit
fi
exec 9>"$RUNTIME/$ROBOT-motion-$UID.lock"
flock -n 9 || { echo '다른 이동 명령 실행 중입니다.' >&2;exit 1; }
if systemctl --user is-active --quiet "$ROBOT-docking.service" || systemctl --user is-active --quiet "$ROBOT-rest.service" || pgrep -u "$USER" -f '[/]nav2_waypoints|[d]ocking_node.py' >/dev/null;then
 echo '기존 이동을 먼저 종료하세요.' >&2;exit 1
fi
# Do not pass the caller's action/waypoint arguments into preparation.
source "$HERE/warm.sh" ""
bash "$HERE/ensure_camera.sh"
bash "$HERE/ensure_docking_ready.sh"

MODE=normal
if [[ "$ACTION" == park ]];then
 MODE=parking
fi
rm -f -- "$ROOT/data/$ROBOT/terminal_started" "$ROOT/data/$ROBOT/terminal_result.json"
systemctl --user reset-failed "$ROBOT-docking.service" 2>/dev/null || true
TOKEN=$(printenv BURGER_MISSION_TOKEN || true)
systemd-run --user --collect --unit="$ROBOT-docking" --property=KillSignal=SIGINT --property=TimeoutStopSec=8 --setenv=ROS_DOMAIN_ID=40 --setenv="BURGER_MISSION_TOKEN=$TOKEN" /bin/bash "$HERE/terminal_wrapper.sh" "$ACTION" "$CAMERA_ROOT/start_docking.sh" --mode "$MODE" --execute --auto-start --exit-on-result
# A submitted systemd job is not yet a running controller.
for (( i=0; i<100; i++ )); do
 if [[ -f "$ROOT/data/$ROBOT/terminal_result.json" ]]; then
  python3 - "$ROOT/data/$ROBOT/terminal_result.json" <<'CHECK'
import json,sys
r=json.load(open(sys.argv[1])); print('도킹/파킹 결과:',r)
sys.exit(0 if r['status']=='success' else 1)
CHECK
  exit $?
 fi
 if [[ -f "$ROOT/data/$ROBOT/terminal_started" ]]; then
  echo "$ROBOT 도킹/파킹 제어권 전환 완료. 컨트롤러 시작 중입니다."
  exit 0
 fi
 if ! systemctl --user is-active --quiet "$ROBOT-docking.service"; then
  journalctl --user -u "$ROBOT-docking.service" -n 8 --no-pager >&2
  exit 1
 fi
 sleep .2
done
echo '도킹/파킹 시작 확인 시간 초과. 로그를 확인하세요.' >&2
exit 1
