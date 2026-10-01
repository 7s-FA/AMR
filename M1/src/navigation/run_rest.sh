#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_ROOT=/home/ubuntu/final_robot_camera_burger1
source /opt/ros/jazzy/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
if [[ "${1:-}" == --check ]]; then
  exec python3 "$HERE/rest_forward.py" --config "$CAMERA_ROOT/docking.yaml" --check
fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/burger1-motion-$UID.lock"
flock -n 9 || { echo '진행 중인 주행을 먼저 종료하세요.' >&2; exit 1; }
if systemctl --user is-active --quiet burger1-docking.service || pgrep -u "$USER" -f '[/]nav2_waypoints|[d]ocking_node.py' >/dev/null; then
  echo '기존 웨이포인트/도킹 주행이 진행 중입니다.' >&2; exit 1
fi
bash "$HERE/warm.sh"
export ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 "$CAMERA_ROOT/docking_network.py" --config "$CAMERA_ROOT/docking.yaml" --role robot --output "$HERE/dds_rest.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/dds_rest.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
exec python3 "$HERE/rest_forward.py" --config "$CAMERA_ROOT/docking.yaml"
