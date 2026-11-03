#!/usr/bin/env bash
# ========================================================================
# 역할: Nav2 실행. burger2-nav2.service 의 본체 → burger2_navigation.launch.py (목표는 보내지 않음).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Robot service entry point. No goals are sent by this script.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../../handoff/pc_burger2_env.bash"
source "$SCRIPT_DIR/../../common/nav2_network.bash"
configure_nav2_network burger2 "$@"
exec ros2 launch waffle_navigation burger2_navigation.launch.py \
  params_file:="$BURGER2_NAV_PARAMS" waypoints_file:="$BURGER2_WAYPOINTS" "$@" use_rviz:=false
