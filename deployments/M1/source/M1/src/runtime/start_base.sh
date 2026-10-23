#!/usr/bin/env bash
# ========================================================================
# 역할: 버거3(STM32 F411 + micro-ROS) 본체 시작. burger1-base.service 가 실행한다. Nav2·속도 명령 없음.
#   1) micro-ROS Agent docker(f411-agent, 펌웨어 통신 921600bps) 시작
#   2) STM32 리셋 (펌웨어는 켜질 때 한 번만 Agent 에 붙으므로 Agent 를 켠 뒤 리셋) — openocd, 플래시 쓰기 없음
#   3) DDS 설정(docking_network.py) 후 base.launch.py 실행 (브리지·odom·라이다·sensor_state 대체 노드)
# 환경 변수: F411_BAUD(기본 921600), F411_DEV(기본 /dev/ttyACM0)
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi
source "$HOME/stm32_bridge_ws/install/setup.bash"
export ROS_DOMAIN_ID="${BURGER1_ROS_DOMAIN_ID:-40}" TURTLEBOT3_MODEL=burger AMR_ROBOT="${AMR_ROBOT:-burger1}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '(/turtlebot3_ros|/ld08_driver|stm32_adapter|stm32_odometry)( |$)' >/dev/null; then
  echo 'A base driver (turtlebot3_ros/ld08/stm32 bridge) is already running. Stop it first.' >&2
  exit 1
fi
if ! docker info >/dev/null 2>&1; then
  echo 'docker 권한 없음: sudo usermod -aG docker ubuntu 후 재로그인/재부팅' >&2
  exit 1
fi
# 1) micro-ROS Agent. 팀원의 디버그용 imu-agent-debug 는 같은 시리얼을 쓰므로 멈춘다.
docker stop imu-agent-debug >/dev/null 2>&1 || true
docker rm -f f411-agent >/dev/null 2>&1 || true
docker run -d --rm --name f411-agent --net=host --privileged -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -v /dev:/dev \
  microros/micro-ros-agent:jazzy serial --dev "${F411_DEV:-/dev/ttyACM0}" -b "${F411_BAUD:-921600}" >/dev/null
sleep 2
# 2) STM32 리셋 (리셋 버튼과 같음, ST-Link USB 접근 권한: plugdev 그룹)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c 'init; reset run; shutdown' 2>&1 | tail -3
# 3) DDS 설정 → 본체 launch
if [[ -f "$SCRIPT_DIR/docking_network.py" && -f "$SCRIPT_DIR/docking.yaml" ]]; then
  /usr/bin/python3 "$SCRIPT_DIR/docking_network.py" --config "$SCRIPT_DIR/docking.yaml" \
    --role robot --output "$SCRIPT_DIR/dds_base.xml"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/dds_base.xml"
fi
exec ros2 launch "$SCRIPT_DIR/base.launch.py" "$@"
