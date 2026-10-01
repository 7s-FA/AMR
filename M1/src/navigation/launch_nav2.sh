#!/usr/bin/env bash
# Robot service entry point. No goals are sent by this script.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../../handoff/pc_burger1_env.bash"
source "$SCRIPT_DIR/../../common/nav2_network.bash"
configure_nav2_network burger1 "$@"
exec ros2 launch waffle_navigation burger1_navigation.launch.py \
  params_file:="$BURGER1_NAV_PARAMS" waypoints_file:="$BURGER1_WAYPOINTS" "$@" use_rviz:=false
