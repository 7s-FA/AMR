# ========================================================================
# 역할: 주행 스크립트 공통 환경. 로봇 ROS 환경 + DDS 네트워크 설정을 한 번만 만들고 재사용한다 (source 용).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Source once per command chain; a new SSH command still refreshes networking.
_burger_env_here=$(cd "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
_burger_env_robot=$(basename "$(dirname "$_burger_env_here")")
_burger_env_root=$(cd "$_burger_env_here/../../.." && pwd)
if [[ "${RMW_IMPLEMENTATION:-}" == rmw_fastrtps_cpp &&
      "${RMW_FASTRTPS_PUBLICATION_MODE:-}" == ASYNCHRONOUS &&
      "${BURGER_NAV_ENV_READY:-}" == "$_burger_env_robot:${ROS_DOMAIN_ID:-}:${FASTRTPS_DEFAULT_PROFILES_FILE:-}" &&
      -r "${FASTRTPS_DEFAULT_PROFILES_FILE:-/nonexistent}" ]]; then
  unset _burger_env_here _burger_env_robot _burger_env_root
  return 0
fi
source "$_burger_env_root/handoff/pc_${_burger_env_robot}_env.bash"
source "$_burger_env_root/robot/common/nav2_network.bash"
configure_nav2_network "$_burger_env_robot" "$@"
# Dry-run may have skipped profile creation; never cache that as ROS readiness.
if [[ -r "${FASTRTPS_DEFAULT_PROFILES_FILE:-/nonexistent}" ]]; then
  export BURGER_NAV_ENV_READY="$_burger_env_robot:${ROS_DOMAIN_ID:-}:${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi
unset _burger_env_here _burger_env_robot _burger_env_root
