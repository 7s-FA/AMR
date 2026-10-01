#!/usr/bin/env bash
# Robot only: save into the onboard params file used by the Nav2 service.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../../handoff/pc_burger2_env.bash"
source "$SCRIPT_DIR/../../common/nav2_network.bash"
configure_nav2_network burger2 "$@"
exec ros2 run waffle_navigation save_start_pose --params-file "$BURGER2_NAV_PARAMS" \
  --namespace burger2 "$@" --ros-args -r /tf:=tf -r /tf_static:=tf_static
