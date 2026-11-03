#!/usr/bin/env bash
# ========================================================================
# 역할: 운영자가 '로봇이 자기 주차 자리에 멈춰 있다'고 확인할 때 실행. 초기 위치 적용(warm.sh --force-parked) 후
#       출차 표시(departure_pending)를 만들어 다음 주행에서 출차를 한 번 하게 한다. 호출: operation.py parked.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Operator explicitly confirms a stationary robot is in its own parking bay.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
source "$HERE/mission_guard.sh";mission_guard "$ROBOT" "${1:-}"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/$ROBOT-motion-$UID.lock"
flock -n 9 || { echo '이동 중에는 주차 확인을 할 수 없습니다.';exit 1; }
if systemctl --user is-active --quiet "$ROBOT-docking.service" || systemctl --user is-active --quiet "$ROBOT-rest.service" || pgrep -u "$USER" -f '[/]nav2_waypoints|[d]ocking_node.py' >/dev/null;then exit 1;fi
# Apply the explicit parking confirmation inside the shared initialization lock.
bash "$HERE/warm.sh" --force-parked
mkdir -p "$HERE/../../../data/$ROBOT"
date -Is > "$HERE/../../../data/$ROBOT/departure_pending"
echo '지정 주차장에 정지해 있다고 확인했습니다. 다음 주행에서 출차를 한 번 실행합니다.'
