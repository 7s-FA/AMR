#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/../.." && pwd)
[[ -f "$ROOT/.test_host_ws/install/setup.bash" ]] || { echo '먼저 tools/test_host/build.sh를 실행하세요.' >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$ROOT/.test_host_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-40}"
exec python3 "$HERE/client.py" "$@"
