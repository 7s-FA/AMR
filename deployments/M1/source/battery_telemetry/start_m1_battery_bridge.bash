#!/usr/bin/env bash
# Install beside m1_battery_bridge.py on M1. Read-only, no motion dependencies.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY ROS_DISCOVERY_SERVER ROS_STATIC_PEERS CYCLONEDDS_URI
unset FASTDDS_DEFAULT_PROFILES_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/AMR/.runtime/M1/final_robot_camera_burger1/dds_base.xml"
if [[ ! -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]]; then
    echo 'M1 DDS profile is missing; prepare the existing base configuration first.' >&2
    exit 1
fi
BATTERY_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# systemd user services can start before Wi-Fi has an address. Fast DDS may
# otherwise stay alive with an unusable transport and never recover on its own.
/usr/bin/python3 "$BATTERY_SCRIPT_DIR/wait_m1_battery_network.py" "$FASTRTPS_DEFAULT_PROFILES_FILE"
exec /usr/bin/python3 "$BATTERY_SCRIPT_DIR/m1_battery_bridge.py"
