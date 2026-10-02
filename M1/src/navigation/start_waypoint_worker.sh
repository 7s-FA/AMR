#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/nav_env.bash"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export BURGER_WAYPOINT_TERMINAL_SPEED=0.01728
export BURGER_WAYPOINT_ENTRY_PATH="$BURGER_PROJECT_ROOT/host_ws/install/waffle_navigation/lib/waffle_navigation/nav2_waypoints"
# Keep the route executable OUT of argv: admission uses pgrep /nav2_waypoints.
exec /usr/bin/python3 -u "$HERE/waypoint_worker.py"
