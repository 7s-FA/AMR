#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
ACTION="$1"
case "$ACTION" in
 stop) bash "$HERE/set_mode.sh" idle || true;exec systemctl --user stop "$ROBOT-rest.service" ;;
 status) exec journalctl --user -u "$ROBOT-rest.service" -n 30 --no-pager ;;
 check) exec bash "$HERE/run_rest.sh" --check ;;
 start) ;;
 *) exit 2 ;;
esac
source "$HERE/mission_guard.sh";mission_guard "$ROBOT"
if systemctl --user is-active --quiet "$ROBOT-docking.service" || pgrep -u "$USER" -f '[/]nav2_waypoints|[d]ocking_node.py' >/dev/null;then
 echo '기존 이동을 먼저 종료하세요.' >&2;exit 1
fi
bash "$HERE/warm.sh"
if systemctl --user is-active --quiet "$ROBOT-rest.service";then echo '이미 휴식 이동 중입니다.' >&2;exit 1;fi
systemctl --user reset-failed "$ROBOT-rest.service" 2>/dev/null || true
TOKEN=$(printenv BURGER_MISSION_TOKEN || true)
systemd-run --user --collect --unit="$ROBOT-rest" --property=KillSignal=SIGINT --property=TimeoutStopSec=8 --setenv=ROS_DOMAIN_ID=40 --setenv="BURGER_MISSION_TOKEN=$TOKEN" /bin/bash "$HERE/terminal_wrapper.sh" rest "$HERE/run_rest.sh"
echo 'IR 직진 이동 요청 전달 완료'
