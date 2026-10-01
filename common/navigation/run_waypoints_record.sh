#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
export BURGER_RECORD_BAG=1
exec bash "$HERE/run_selected_waypoints.sh" 12 "$@"
