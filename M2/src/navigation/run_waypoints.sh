#!/usr/bin/env bash
# Robot only. Without --dry-run this commands movement.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/nav_env.bash" "$@"
# Burger2 waypoint final approach: 0.03 m/s reduced by 20%; departure unchanged.
export BURGER_WAYPOINT_TERMINAL_SPEED=0.024
WAYPOINT_ENTRY="$BURGER_PROJECT_ROOT/host_ws/install/waffle_navigation/lib/waffle_navigation/nav2_waypoints"
if [[ ! -f "$WAYPOINT_ENTRY" ]];then echo "Missing waypoint program: $WAYPOINT_ENTRY" >&2;exit 1;fi
exec /usr/bin/python3 "$WAYPOINT_ENTRY" --namespace burger2 --waypoints "$BURGER2_WAYPOINTS" \
  --nav2-position-then-yaw "$@" \
  --ros-args -r /tf:=tf -r /tf_static:=tf_static
