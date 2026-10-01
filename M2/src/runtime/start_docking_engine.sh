#!/usr/bin/env bash
set -eo pipefail
# Bound library thread pools before importing NumPy/OpenCV in spawned workers.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Standard normal docking reuses preloaded imports and fresh detection.
if [[ "$*" == '--mode normal --execute --auto-start --exit-on-result' ]]; then
 if /usr/bin/python3 "$SCRIPT_DIR/docking_warm_client.py" burger2 --check; then
  exec /usr/bin/python3 "$SCRIPT_DIR/docking_warm_client.py" burger2
 fi
fi
# A custom/cold path must not share a preview with the standby detector.
if systemctl --user is-active --quiet burger2-docking-ready.service;then
 systemctl --user stop burger2-docking-ready.service
fi
source /opt/ros/jazzy/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-40}"
/usr/bin/python3 "$SCRIPT_DIR/docking_network.py" --config "$SCRIPT_DIR/docking.yaml" \
  --role robot --local-shm --output "$SCRIPT_DIR/dds_robot.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/dds_robot.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
# Support the project checkout as well as the flat deployment directory.
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
DATA_ARGS=()
if [[ -d "$PROJECT_ROOT/host_ws/src/docking_vision/docking_vision" ]]; then
  export PYTHONPATH="$PROJECT_ROOT/host_ws/src/docking_vision:${PYTHONPATH:-}"
  DATA_ARGS=(--calibration "$PROJECT_ROOT/data/burger2/calibration/camera.yaml")
fi
exec /usr/bin/python3 "$SCRIPT_DIR/docking_node.py" --config "$SCRIPT_DIR/docking.yaml" "${DATA_ARGS[@]}" "$@"
