#!/usr/bin/env bash
# Operator explicitly confirms a stationary robot is in its own parking bay.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
source "$HERE/mission_guard.sh";mission_guard "$ROBOT"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/$ROBOT-motion-$UID.lock"
flock -n 9 || { echo '이동 중에는 주차 확인을 할 수 없습니다.';exit 1; }
if systemctl --user is-active --quiet "$ROBOT-docking.service" || systemctl --user is-active --quiet "$ROBOT-rest.service" || pgrep -u "$USER" -f '[/]nav2_waypoints|[d]ocking_node.py' >/dev/null;then exit 1;fi
# Apply the explicit parking confirmation inside the shared initialization lock.
bash "$HERE/warm.sh" --force-parked
mkdir -p "$HERE/../../../data/$ROBOT"
date -Is > "$HERE/../../../data/$ROBOT/departure_pending"
echo '지정 주차장에 정지해 있다고 확인했습니다. 다음 주행에서 출차를 한 번 실행합니다.'
