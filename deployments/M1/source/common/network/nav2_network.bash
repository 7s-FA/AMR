# ========================================================================
# 역할: configure_nav2_network 함수. docking.yaml 의 Wi-Fi 인터페이스로 DDS 프로필(data/<로봇>/dds_nav2.xml)을 만들고
#       로컬 공유메모리 전송을 추가해 FASTRTPS_DEFAULT_PROFILES_FILE 로 지정한다. 사용처: launch_*.sh, nav_env.bash.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Sourced after handoff/pc_burger{1,2}_env.bash on the robot.
# Match the base's UDP-only interface profile; do not start any ROS process.
configure_nav2_network() {
  local robot_id="$1"
  shift
  local argument
  for argument in "$@"; do
    # Host-side coordinate validation must work without the robot's interface.
    if [[ "$argument" == --dry-run ]]; then
      return 0
    fi
  done
  if [[ "$robot_id" != burger1 && "$robot_id" != burger2 ]]; then
    echo "Unsupported Nav2 robot: $robot_id" >&2
    return 1
  fi
  local default_base_dir="${AMR_CAMERA}"
  if [[ "$robot_id" == burger2 ]]; then
    default_base_dir="${AMR_CAMERA}"
  fi
  local base_dir="${BURGER_NAV2_BASE_DIR:-$default_base_dir}"
  if [[ ! -f "$base_dir/docking_network.py" || ! -f "$base_dir/docking.yaml" ]]; then
    base_dir="$BURGER_PROJECT_ROOT/robot/$robot_id"
  fi
  if [[ ! -f "$base_dir/docking_network.py" || ! -f "$base_dir/docking.yaml" ]]; then
    echo "Nav2 requires the base network config and generator: $base_dir" >&2
    return 1
  fi
  local output_dir="$BURGER_PROJECT_ROOT/data/$robot_id"
  mkdir -p "$output_dir" || return 1
  local profile="$output_dir/dds_nav2.xml"
  local temporary
  temporary="$(mktemp "$profile.XXXXXX")" || return 1
  if ! /usr/bin/python3 "$base_dir/docking_network.py" --config "$base_dir/docking.yaml" \
      --role robot --output "$temporary"; then
    rm -f "$temporary"
    return 1
  fi
  if ! /usr/bin/python3 "$BURGER_PROJECT_ROOT/robot/common/nav2_local_transport.py" "$temporary"; then
    rm -f "$temporary"
    return 1
  fi
  if ! mv -f "$temporary" "$profile"; then
    rm -f "$temporary"
    return 1
  fi
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
  export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  unset FASTDDS_DEFAULT_PROFILES_FILE ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
}
