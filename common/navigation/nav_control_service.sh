#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
source "$HERE/nav_env.bash"
exec python3 "$HERE/motion_mode.py" "$ROBOT" --serve
