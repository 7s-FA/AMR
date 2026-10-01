#!/usr/bin/env bash
# Robot service entry point. No goals are sent by this script.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../../handoff/pc_burger2_env.bash"
source "$SCRIPT_DIR/../../common/nav2_network.bash"
configure_nav2_network burger2 "$@"
exec ros2 launch waffle_navigation burger2_navigation.launch.py \
  params_file:="$BURGER2_NAV_PARAMS" waypoints_file:="$BURGER2_WAYPOINTS" "$@" use_rviz:=false
