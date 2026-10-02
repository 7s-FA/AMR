#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
CAMERA_ROOT=/home/ubuntu/final_robot_camera
[[ "$ROBOT" != burger1 ]] || CAMERA_ROOT=/home/ubuntu/final_robot_camera_burger1
ACTION=$1
case "$ACTION" in
 status) exec systemctl --user status "$ROBOT-base.service" "$ROBOT-localization.service" "$ROBOT-nav2.service" "$ROBOT-motion-owner.service" "$ROBOT-nav-control.service" "$ROBOT-docking.service" "$ROBOT-rest.service" --no-pager ;;
 stop|off|dock-stop)
  bash "$HERE/set_mode.sh" idle || true
  systemctl --user kill --kill-whom=main --signal=USR1 burger2-rest-ready.service 2>/dev/null || true
  pkill -INT -u "$USER" -f '[/]nav2_waypoints' || true
  # Transient units may never have existed or have already been collected.
  # That is already stopped, not a failed mission cleanup.
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
exec 9>"$RUNTIME/$ROBOT-motion-$UID.lock"
flock -n 9 || { echo '다른 이동 명령 실행 중입니다.' >&2;exit 1; }
if systemctl --user is-active --quiet "$ROBOT-docking.service" || systemctl --user is-active --quiet "$ROBOT-rest.service" || pgrep -u "$USER" -f '[/]nav2_waypoints|[d]ocking_node.py' >/dev/null;then
 echo '기존 이동을 먼저 종료하세요.' >&2;exit 1
fi
# Do not pass the caller's action/waypoint arguments into preparation.
source "$HERE/warm.sh" ""
bash "$HERE/ensure_camera.sh"
if [[ "$ACTION" == ready || "$ACTION" == dock || "$ACTION" == park ]];then bash "$HERE/ensure_docking_ready.sh";fi
if [[ "$ACTION" == ready ]];then
 bash "$HERE/set_mode.sh" prepare
 echo "$ROBOT 준비 완료. 지정 주차장 시작 위치 자동 적용 / 이후 위치 추정 유지.";exit;fi

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
