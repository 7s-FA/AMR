#!/usr/bin/env bash
set -eo pipefail
[[ $# == 1 ]] || { echo 'Usage: build.sh RUNTIME_DIRECTORY'; exit 2; }
RUNTIME=$(realpath "$1")
[[ -f "$RUNTIME/runtime.json" ]] || { echo 'Run materialize.py first'; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
cd "$RUNTIME/final_robot_ws/host_ws"
colcon build --symlink-install --packages-up-to waffle_navigation amr_mission --parallel-workers 2
