#!/usr/bin/env bash
# Camera preparation only: no GPIO or motor commands.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
CAMERA_ROOT=$HOME/final_robot_camera
[[ "$ROBOT" != burger1 ]] || CAMERA_ROOT=$HOME/final_robot_camera_burger1
if ! systemctl --user is-active --quiet "$ROBOT-camera.service"; then
  systemctl --user reset-failed "$ROBOT-camera.service" 2>/dev/null || true
  systemd-run --user --collect --unit="$ROBOT-camera" --property=KillSignal=SIGINT --property=TimeoutStopSec=5 --setenv=ROS_DOMAIN_ID=40 /bin/bash "$CAMERA_ROOT/start_camera.sh"
fi
