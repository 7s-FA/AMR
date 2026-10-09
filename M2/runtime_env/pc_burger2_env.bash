# ========================================================================
# 역할: burger2 ROS 환경 설정 (pc_env.bash + Nav2 파라미터·도메인 경로). launch_*.sh, nav_env.bash 가 source.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Robot-local ROS environment; assembled by materialize.py.
source "$(dirname -- "${BASH_SOURCE[0]}")/pc_env.bash"
export BURGER2_NAV_PARAMS="$BURGER_PROJECT_ROOT/host_ws/src/waffle_navigation/config/nav2_burger2_params.yaml"
export BURGER2_ROS_DOMAIN_ID="${BURGER2_ROS_DOMAIN_ID:-40}"
export ROS_DOMAIN_ID="$BURGER2_ROS_DOMAIN_ID"
export BURGER2_WAYPOINTS="$BURGER_PROJECT_ROOT/host_ws/src/waffle_navigation/config/waypoints_burger2.yaml"
