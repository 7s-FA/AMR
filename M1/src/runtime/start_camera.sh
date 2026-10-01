#!/usr/bin/env bash
# Full-field IMX708 capture; recalibrate using the published 640x360 images.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${BURGER1_ROS_DOMAIN_ID:-40}"
/usr/bin/python3 "$SCRIPT_DIR/docking_network.py" --config "$SCRIPT_DIR/docking.yaml" \
  --role robot --local-shm --output "$SCRIPT_DIR/dds_camera.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/dds_camera.xml"
exec /usr/bin/python3 "$SCRIPT_DIR/camera_node.py" --config "$SCRIPT_DIR/docking.yaml" "$@"
