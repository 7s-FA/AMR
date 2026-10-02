#!/usr/bin/env bash
# Prepare connections only; never send a route or motor command.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ENTRY="$HERE/../../../host_ws/install/waffle_navigation/lib/waffle_navigation/nav2_waypoints"
ENTRY=$(readlink -f "$ENTRY")
if python3 "$HERE/waypoint_client.py" "$ENTRY" --check; then exit 0; fi
systemctl --user start burger1-waypoint-ready.service
# Cold startup is bounded. Existing per-route code remains the safe fallback.
for ((i=0; i<100; i++)); do
  if python3 "$HERE/waypoint_client.py" "$ENTRY" --check; then exit 0; fi
  systemctl --user is-active --quiet burger1-waypoint-ready.service || exit 1
  sleep .1
done
echo '웨이포인트 사전 준비 시간 초과; 기존 실행 경로를 사용합니다.' >&2
exit 1
