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
