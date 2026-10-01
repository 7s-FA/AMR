#!/usr/bin/env bash
# Warm imports and camera detection only; no GPIO or velocity commands.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
CAMERA_ROOT=$HOME/final_robot_camera
bash "$HERE/ensure_camera.sh"
if ! systemctl --user is-active --quiet "$ROBOT-docking-ready.service";then
 systemctl --user reset-failed "$ROBOT-docking-ready.service" 2>/dev/null || true
 systemd-run --user --quiet --collect --unit="$ROBOT-docking-ready" --property=KillSignal=SIGINT --property=TimeoutStopSec=5 --property=Restart=on-failure --property=RestartSec=2 --setenv=ROS_DOMAIN_ID=40 /bin/bash "$CAMERA_ROOT/start_docking_standby.sh"
fi
