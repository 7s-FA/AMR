#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
# Readiness/rate change only. The engine still owns all control/safety checks.
bash "$HERE/camera_profile.sh" active
cleanup_camera() { status=$?;trap - EXIT;bash "$HERE/camera_profile.sh" idle || true;exit "$status"; }
trap cleanup_camera EXIT
bash "$HERE/start_docking_engine.sh" "$@"
