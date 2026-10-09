#!/usr/bin/env bash
# ========================================================================
# 역할: 임무 1건의 시작 스크립트. 임무 잠금·소유 토큰을 만든 뒤 sequence_runner.py 실행.
# 호출: operation.submit 이 systemd-run 으로 burger1-mission 서비스로 실행.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT=burger1
exec 8>"${XDG_RUNTIME_DIR:-/tmp}/$ROBOT-mission-$UID.lock"
flock -n 8 || { echo '이동 시퀀스가 이미 실행 중입니다.' >&2; exit 1; }
export BURGER_MISSION_TOKEN="$$-$(date +%s%N)"
OWNER="${XDG_RUNTIME_DIR:-/tmp}/$ROBOT-mission.owner"
printf '%s' "$BURGER_MISSION_TOKEN" > "$OWNER"
trap 'rm -f -- "$OWNER"' EXIT
python3 "$HERE/sequence_runner.py" "$@" --robot "$ROBOT"
