# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Robot-only ROS environment generated for M1.
source "${AMR_WORKSPACE}/handoff/ros_setup_fast.bash" || return 1
export BURGER_PROJECT_ROOT="${AMR_WORKSPACE}" BURGER_NAV2_BASE_DIR="${AMR_CAMERA}"
export ROS_DOMAIN_ID=40 TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
