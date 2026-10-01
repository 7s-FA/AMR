#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
exec python3 "$HERE/motion_client.py" burger1 "$1"
