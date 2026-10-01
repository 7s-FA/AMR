#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/../../../handoff/pc_burger2_env.bash"
source "$HERE/../../common/nav2_network.bash"
configure_nav2_network burger2
exec python3 "$HERE/motion_owner.py" --robot burger2
