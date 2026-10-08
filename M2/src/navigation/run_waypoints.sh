#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Robot only. Without --dry-run this commands movement.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/nav_env.bash" "$@"
# Burger2 waypoint final approach: 0.03 m/s reduced by 20%; departure unchanged.
export BURGER_WAYPOINT_TERMINAL_SPEED=0.01728
WAYPOINT_ENTRY="$BURGER_PROJECT_ROOT/host_ws/install/waffle_navigation/lib/waffle_navigation/nav2_waypoints"
if [[ ! -f "$WAYPOINT_ENTRY" ]];then echo "Missing waypoint program: $WAYPOINT_ENTRY" >&2;exit 1;fi
# An already connected worker changes startup only. It runs the SAME main/args.
# Exit 2 means no request was sent; only that case permits cold fallback.
status=0
/usr/bin/python3 "$SCRIPT_DIR/waypoint_client.py" "$WAYPOINT_ENTRY" --namespace burger2 --waypoints "$BURGER2_WAYPOINTS" \
  --nav2-position-then-yaw "$@" \
  --ros-args -r /tf:=tf -r /tf_static:=tf_static || status=$?
if (( status != 2 )); then exit "$status"; fi
exec /usr/bin/python3 "$WAYPOINT_ENTRY" --namespace burger2 --waypoints "$BURGER2_WAYPOINTS" \
  --nav2-position-then-yaw "$@" \
  --ros-args -r /tf:=tf -r /tf_static:=tf_static
