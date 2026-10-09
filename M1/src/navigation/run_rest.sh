#!/usr/bin/env bash
# ========================================================================
# 역할: REST 종단 동작(IR 직진 정지) 실행. 대기 작업자가 있으면 그쪽으로 요청, 없을 때만 rest_forward.py 직접 실행.
# 호출: rest.sh start 가 burger1-rest 서비스로 실행. --check 는 설정 확인만.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_ROOT=${AMR_CAMERA}
if [[ "${1:-}" == --check ]]; then
  exec python3 "$HERE/rest_forward.py" --config "$CAMERA_ROOT/docking.yaml" --check
fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/burger1-motion-$UID.lock"
flock -n 9 || { echo '진행 중인 주행을 먼저 종료하세요.' >&2; exit 1; }
if systemctl --user is-active --quiet burger1-docking.service || pgrep -u "$USER" -f '[/]nav2_waypoints|[d]ocking_node.py' >/dev/null; then
  echo '기존 웨이포인트/도킹 주행이 진행 중입니다.' >&2; exit 1
fi
status=0
BURGER_PREPARED_SOCKET_NAME=burger1-rest-ready.sock /usr/bin/python3 "$HERE/waypoint_client.py" "$HERE/rest_forward.py" --config "$CAMERA_ROOT/docking.yaml" || status=$?
# Only 2 (no request sent) permits the original cold execution. Never duplicate an accepted move.
if (( status != 2 )); then exit "$status"; fi
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi
bash "$HERE/warm.sh"
export ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 "$CAMERA_ROOT/docking_network.py" --config "$CAMERA_ROOT/docking.yaml" --role robot --output "$HERE/dds_rest.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/dds_rest.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
exec python3 "$HERE/rest_forward.py" --config "$CAMERA_ROOT/docking.yaml"
