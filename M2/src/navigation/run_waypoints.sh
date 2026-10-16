#!/usr/bin/env bash
# ========================================================================
# 역할: nav2_waypoints 실행기. 대기 작업자(waypoint_client)를 먼저 쓰고, 요청을 못 보냈을 때(코드 2)만 직접 실행.
# 호출: run_selected_waypoints.sh, sequence_runner.py(도킹 재시도 후진).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Robot only. Without --dry-run this commands movement.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/nav_env.bash" "$@"
# Burger2 waypoint final approach: 0.03 m/s reduced by 20%; departure unchanged.
export BURGER_WAYPOINT_TERMINAL_SPEED=0.01728
WAYPOINT_ENTRY="$BURGER_PROJECT_ROOT/host_ws/install/waffle_navigation/lib/waffle_navigation/nav2_waypoints"
if [[ ! -f "$WAYPOINT_ENTRY" ]];then echo "Missing waypoint program: $WAYPOINT_ENTRY" >&2;exit 1;fi
# An already connected worker changes startup only. It runs the SAME main/args.
# Exit 2 means no request was sent; only that case permits cold fallback.
status=0
/usr/bin/python3 "$SCRIPT_DIR/waypoint_client.py" "$WAYPOINT_ENTRY" --namespace burger2 --waypoints "$BURGER2_WAYPOINTS" \
  --nav2-position-then-yaw "$@" \
  --ros-args -r /tf:=tf -r /tf_static:=tf_static || status=$?
if (( status != 2 )); then exit "$status"; fi
exec /usr/bin/python3 "$WAYPOINT_ENTRY" --namespace burger2 --waypoints "$BURGER2_WAYPOINTS" \
  --nav2-position-then-yaw "$@" \
  --ros-args -r /tf:=tf -r /tf_static:=tf_static
