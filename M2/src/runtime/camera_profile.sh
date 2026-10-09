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
ROBOT=burger2
PROFILE="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/$ROBOT-camera-profile"
case "${1:-}" in active|idle) MODE=$1;;*) exit 2;;esac
exec 6>"$PROFILE.lock"
flock -w 10 6
# Native control is independent of docking standby configuration.
apply_profile() {
 /usr/bin/python3 - "$HERE" "$ROBOT" "$MODE" <<'PY'
import json, sys, time
sys.path.insert(0, sys.argv[1])
from camera_ipc import profile
robot, mode = sys.argv[2:4]
for attempt in range(3):
    try:
        report = profile(robot, mode, timeout=2.)
    except (OSError, RuntimeError) as exc:
        if isinstance(exc, RuntimeError) and str(exc) != 'Camera control disconnected':
            print('Camera profile rejected: ' + str(exc), file=sys.stderr)
            sys.exit(3)
        print(f'Camera control attempt {attempt+1}/3 failed: {exc}', file=sys.stderr)
        if attempt == 2:
            sys.exit(2)
        time.sleep(.2)
        continue
    except Exception as exc:
        print('Camera profile rejected: ' + str(exc), file=sys.stderr)
        sys.exit(3)
    if report.get('camera_profile') != mode or report.get('frame_duration_us') != (66667 if mode == 'active' else 500000):
        print('Camera profile acknowledgement mismatch: ' + str(report), file=sys.stderr)
        sys.exit(3)
    print(json.dumps(report), flush=True)
    break
PY
}
systemctl --user start "$ROBOT-camera.service"
status=0
apply_profile || status=$?
if (( status == 2 )); then
 echo 'Camera control unavailable; restarting once and reapplying profile.' >&2
 systemctl --user restart "$ROBOT-camera.service"
 apply_profile
elif (( status != 0 )); then
 exit "$status"
fi
# Always request the actual mode, even if the local record already matches.
printf '%s\n' "$MODE" > "$PROFILE.tmp";mv "$PROFILE.tmp" "$PROFILE"
