#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Warm imports and camera detection only; no GPIO or velocity commands.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
CAMERA_ROOT=${AMR_CAMERA}
bash "$HERE/ensure_camera.sh"
if [[ ! $(systemctl --user show "$ROBOT-docking-ready.service" -p ActiveState --value) =~ ^(active|activating|reloading)$ ]];then
 systemctl --user reset-failed "$ROBOT-docking-ready.service" 2>/dev/null || true
 systemd-run --user --quiet --collect --unit="$ROBOT-docking-ready" --property=KillSignal=SIGINT --property=TimeoutStopSec=5 --property=Restart=on-failure --property=RestartSec=2 --setenv=ROS_DOMAIN_ID=40 /bin/bash "$CAMERA_ROOT/start_docking_standby.sh"
fi
