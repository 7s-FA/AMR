#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${BURGER1_ROS_DOMAIN_ID:-40}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/ir_sensor.py" --chip /dev/gpiochip4 --pin 17 "$@"
