#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Motor/odometry driver only; this does not start Nav2 or issue velocity commands.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi
export ROS_DOMAIN_ID="${BURGER1_ROS_DOMAIN_ID:-40}"
export TURTLEBOT3_MODEL=burger
export LDS_MODEL="${LDS_MODEL:-LDS-02}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '(/turtlebot3_ros|/ld08_driver)( |$)' >/dev/null; then
  echo 'A TurtleBot3/OpenCR or LDS-02 driver is already running. Stop its launch/service before starting this base.' >&2
  exit 1
fi
if [[ -f "$SCRIPT_DIR/docking_network.py" && -f "$SCRIPT_DIR/docking.yaml" ]]; then
  /usr/bin/python3 "$SCRIPT_DIR/docking_network.py" --config "$SCRIPT_DIR/docking.yaml" \
    --role robot --output "$SCRIPT_DIR/dds_base.xml"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/dds_base.xml"
fi
exec ros2 launch "$SCRIPT_DIR/base.launch.py" "$@"
