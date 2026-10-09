#!/usr/bin/env bash
# ========================================================================
# 역할: 웨이포인트 대기 작업자(waypoint_worker.py) 실행. burger1-waypoint-ready.service 의 본체.
#       경로 코드 경로는 환경변수로 넘긴다 (명령줄에 nav2_waypoints 가 보이면 '주행 중'으로 오인되므로).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/nav_env.bash"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export BURGER_WAYPOINT_TERMINAL_SPEED=0.01728
export BURGER_WAYPOINT_ENTRY_PATH="$BURGER_PROJECT_ROOT/host_ws/install/waffle_navigation/lib/waffle_navigation/nav2_waypoints"
# Keep the route executable OUT of argv: admission uses pgrep /nav2_waypoints.
exec /usr/bin/python3 -u "$HERE/waypoint_worker.py"
