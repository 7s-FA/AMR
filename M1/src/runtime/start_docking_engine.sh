#!/usr/bin/env bash
set -eo pipefail
# Bound library thread pools before importing NumPy/OpenCV in spawned workers.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_ROOT="${BURGER1_DOCK_LOG_ROOT:-$HOME/final_robot_ws/data/burger1/docking_logs}"
RUN_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$RUN_DIR"
ln -sfn "$(basename "$RUN_DIR")" "$LOG_ROOT/latest"
exec > >(tee -a "$RUN_DIR/runtime.log") 2>&1
echo "Docking diagnostics: $RUN_DIR"
# Snapshot process/service state without changing robot operation.
systemctl --user show burger1-base.service burger1-camera.service burger1-nav2.service \
  -p Id -p ActiveState -p SubState -p MainPID > "$RUN_DIR/services.txt" 2>&1 || true
pgrep -af '[c]amera_node.py|[/]cam --|[d]ocking_node.py|[t]urtlebot3_ros' \
  > "$RUN_DIR/processes.txt" || true
# Standard normal docking reuses preloaded imports and fresh detection.
if [[ "$*" == '--mode normal --execute --auto-start --exit-on-result' ]]; then
 if /usr/bin/python3 "$SCRIPT_DIR/docking_warm_client.py" burger1 --check; then
  exec /usr/bin/python3 "$SCRIPT_DIR/docking_warm_client.py" burger1 --log-dir "$RUN_DIR"
 fi
fi
# A custom/cold path must not share a preview with the standby detector.
if systemctl --user is-active --quiet burger1-docking-ready.service;then
 systemctl --user stop burger1-docking-ready.service
fi
source /opt/ros/jazzy/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS_DOMAIN_ID="${BURGER1_ROS_DOMAIN_ID:-40}"
/usr/bin/python3 "$SCRIPT_DIR/docking_network.py" --config "$SCRIPT_DIR/docking.yaml" \
  --role robot --local-shm --output "$SCRIPT_DIR/dds_robot.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/dds_robot.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
# Run directly from the checkout, or from the flat Host deployment directory.
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
DATA_ARGS=()
if [[ -d "$PROJECT_ROOT/host_ws/src/docking_vision/docking_vision" ]]; then
  export PYTHONPATH="$PROJECT_ROOT/host_ws/src/docking_vision:${PYTHONPATH:-}"
  DATA_ARGS=(--calibration "$PROJECT_ROOT/data/burger1/calibration/camera.yaml")
fi
exec /usr/bin/python3 "$SCRIPT_DIR/docking_node.py" --config "$SCRIPT_DIR/docking.yaml" "${DATA_ARGS[@]}" "$@" --log-dir "$RUN_DIR"
