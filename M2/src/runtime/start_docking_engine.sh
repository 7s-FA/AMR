#!/usr/bin/env bash
# ========================================================================
# 역할: 도킹 실행. 대기 작업자(docking_standby)가 있으면 docking_warm_client 로 요청(빠른 경로),
#       없으면 대기 작업자를 멈추고 docking_node.py 를 직접 실행. 로그: data/<로봇>/docking_logs/<시각>/
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
# Normal and parking reuse one detector/camera subscription; fresh mode handshake.
WARM_MODE=""
case "$*" in
 '--mode normal --execute --auto-start --exit-on-result') WARM_MODE=normal ;;
 '--mode parking --execute --auto-start --exit-on-result') WARM_MODE=parking ;;
esac
if [[ -n "$WARM_MODE" ]];then
 if /usr/bin/python3 "$SCRIPT_DIR/docking_warm_client.py" burger2 --mode "$WARM_MODE" --check;then
  exec /usr/bin/python3 "$SCRIPT_DIR/docking_warm_client.py" burger2 --mode "$WARM_MODE" --log-dir "$RUN_DIR"
 fi
fi
# A custom/cold path must not share a preview with the standby detector.
if systemctl --user is-active --quiet burger2-docking-ready.service;then
 systemctl --user stop burger2-docking-ready.service
fi
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
exec /usr/bin/python3 "$SCRIPT_DIR/docking_node.py" --config "$SCRIPT_DIR/docking.yaml" "${DATA_ARGS[@]}" "$@" --log-dir "$RUN_DIR"
