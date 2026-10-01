#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/../.." && pwd)
source /opt/ros/jazzy/setup.bash
mkdir -p "$ROOT/.test_host_ws"
cd "$ROOT/.test_host_ws"
colcon build --base-paths "$ROOT/interfaces/host_pkg" --packages-select host_pkg --symlink-install
