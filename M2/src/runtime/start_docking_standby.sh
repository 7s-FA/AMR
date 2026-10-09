#!/usr/bin/env bash
# ========================================================================
# 역할: 도킹 대기 작업자(docking_standby.py) 실행. burger2-docking-ready 서비스의 본체.
#       로그 폴더: data/<로봇>/docking_logs/<시각>/ (시작할 때마다 하나씩 생김).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
# Bound library thread pools before importing NumPy/OpenCV in spawned workers.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_ROOT="${BURGER2_DOCK_LOG_ROOT:-${AMR_WORKSPACE}/data/burger2/docking_logs}"
RUN_DIR="$LOG_ROOT/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$RUN_DIR"
ln -sfn "$(basename "$RUN_DIR")" "$LOG_ROOT/latest"
exec > >(tee -a "$RUN_DIR/runtime.log") 2>&1
echo "Docking diagnostics: $RUN_DIR"
# Snapshot process/service state without changing robot operation.
systemctl --user show burger2-base.service burger2-camera.service burger2-nav2.service \
  -p Id -p ActiveState -p SubState -p MainPID > "$RUN_DIR/services.txt" 2>&1 || true
pgrep -af '[c]amera_node.py|[/]cam --|[d]ocking_node.py|[t]urtlebot3_ros' \
  > "$RUN_DIR/processes.txt" || true
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi
export ROS_DOMAIN_ID="${BURGER2_ROS_DOMAIN_ID:-40}"
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
  DATA_ARGS=(--calibration "$PROJECT_ROOT/data/burger2/calibration/camera.yaml")
fi
exec /usr/bin/python3 "$SCRIPT_DIR/docking_standby.py" --config "$SCRIPT_DIR/docking.yaml" --robot burger2 "${DATA_ARGS[@]}"
