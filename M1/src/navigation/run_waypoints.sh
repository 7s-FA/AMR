#!/usr/bin/env bash
# Robot only. Without --dry-run this commands movement.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/nav_env.bash" "$@"
exec ros2 run waffle_navigation nav2_waypoints --namespace burger1 --waypoints "$BURGER1_WAYPOINTS" \
  --nav2-position-then-yaw "$@" \
  --ros-args -r /tf:=tf -r /tf_static:=tf_static
