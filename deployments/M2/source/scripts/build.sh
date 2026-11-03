#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
[[ $# == 1 ]] || { echo 'Usage: build.sh RUNTIME_DIRECTORY'; exit 2; }
RUNTIME=$(realpath "$1")
[[ -f "$RUNTIME/runtime.json" ]] || { echo 'Run materialize.py first'; exit 1; }
source "$RUNTIME/runtime.env"
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi
cd "$RUNTIME/final_robot_ws/host_ws"
colcon build --symlink-install --packages-up-to waffle_navigation amr_mission --parallel-workers 2
python3 "$SCRIPT_DIR/repair_install_links.py" "$PWD"
