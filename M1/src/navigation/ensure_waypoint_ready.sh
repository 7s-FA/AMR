#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
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
