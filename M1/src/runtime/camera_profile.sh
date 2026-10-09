#!/usr/bin/env bash
# ========================================================================
# 역할: 카메라 속도 전환 (active=도킹 15fps / idle=대기 2fps). 카메라 프로세스는 유지한 채 설정만 바꾼다.
# 호출: start_docking.sh (시작 시 active, 끝나면 idle).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Camera only: preserve optics/calibration; no GPIO or motion commands.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=burger1
NAV="${AMR_WORKSPACE}/robot/$ROBOT/navigation"
PROFILE="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/$ROBOT-camera-profile"
case "${1:-}" in active|idle) MODE=$1;;*) exit 2;;esac
exec 6>"$PROFILE.lock"
flock -w 10 6
OLD=$(cat "$PROFILE" 2>/dev/null || echo idle)
if [[ ! -r "$PROFILE" || "$OLD" != "$MODE" ]];then
 printf '%s
' "$MODE" > "$PROFILE.tmp";mv "$PROFILE.tmp" "$PROFILE"
 if systemctl --user is-active --quiet "$ROBOT-camera.service";then
  # Dynamic libcamera controls preserve the capture process and image connection.
  if ! /usr/bin/python3 "$HERE/docking_warm_client.py" "$ROBOT" --profile "$MODE";then
    systemctl --user restart "$ROBOT-camera.service"
  fi
 fi
fi
bash "$NAV/ensure_camera.sh"
