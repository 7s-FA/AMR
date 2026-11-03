#!/usr/bin/env bash
# ========================================================================
# 역할: 웨이포인트 대기 작업자(burger1-waypoint-ready)를 확인·시작 (경로·모터 명령 없음).
#       10초 안에 준비 안 되면 실패를 알리고, 주행은 기존 직접 실행 경로로 동작한다.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Prepare connections only; never send a route or motor command.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ENTRY="$HERE/../../../host_ws/install/waffle_navigation/lib/waffle_navigation/nav2_waypoints"
ENTRY=$(realpath -s "$ENTRY")
if python3 "$HERE/waypoint_client.py" "$ENTRY" --check; then exit 0; fi
systemctl --user start burger1-waypoint-ready.service
# Cold startup is bounded. Existing per-route code remains the safe fallback.
if python3 "$HERE/waypoint_client.py" "$ENTRY" --wait-ready 10; then exit 0; fi
echo '웨이포인트 사전 준비 시간 초과; 기존 실행 경로를 사용합니다.' >&2
exit 1
