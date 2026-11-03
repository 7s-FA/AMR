#!/usr/bin/env bash
# ========================================================================
# 역할: 카메라 서비스가 꺼져 있으면 켠다 (GPIO·모터 없음).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Camera preparation only: no GPIO or motor commands.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
CAMERA_ROOT=${AMR_CAMERA}
if [[ ! $(systemctl --user show "$ROBOT-camera.service" -p ActiveState --value) =~ ^(active|activating|reloading)$ ]]; then
  systemctl --user reset-failed "$ROBOT-camera.service" 2>/dev/null || true
  # Prefer deployed persistent units; creating a transient unit with the same name fails.
  _service_fragment=$(systemctl --user show "$ROBOT-camera.service" -p FragmentPath --value 2>/dev/null || true)
  if [[ -n "$_service_fragment" ]]; then
   systemctl --user start "$ROBOT-camera.service"
  else
   systemd-run --user --collect --unit="$ROBOT-camera" --property=KillSignal=SIGINT --property=TimeoutStopSec=5 --setenv=ROS_DOMAIN_ID=40 /bin/bash "$CAMERA_ROOT/start_camera.sh"
  fi
  unset _service_fragment
fi
