#!/usr/bin/env bash
# ========================================================================
# 역할: 위치추정(map_server·AMCL) 실행. burger1-localization.service 의 본체.
#       DDS 네트워크 설정 생성 후 localization_only.launch.py 실행.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/../../../handoff/pc_burger1_env.bash"
source "$HERE/../../common/nav2_network.bash"
configure_nav2_network burger1
MAP="$(ros2 pkg prefix --share waffle_navigation)/maps/factory_map.yaml"
exec ros2 launch "$HERE/localization_only.launch.py" robot:=burger1 params_file:="$BURGER1_NAV_PARAMS" map:="$MAP"
