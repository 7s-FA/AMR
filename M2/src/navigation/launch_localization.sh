#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/../../../handoff/pc_burger2_env.bash"
source "$HERE/../../common/nav2_network.bash"
configure_nav2_network burger2
MAP="$(ros2 pkg prefix --share waffle_navigation)/maps/factory_map.yaml"
exec ros2 launch "$HERE/localization_only.launch.py" robot:=burger2 params_file:="$BURGER2_NAV_PARAMS" map:="$MAP"
