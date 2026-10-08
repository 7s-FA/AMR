#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT=burger2
exec 8>"${XDG_RUNTIME_DIR:-/tmp}/$ROBOT-mission-$UID.lock"
flock -n 8 || { echo '이동 시퀀스가 이미 실행 중입니다.' >&2; exit 1; }
export BURGER_MISSION_TOKEN="$$-$(date +%s%N)"
OWNER="${XDG_RUNTIME_DIR:-/tmp}/$ROBOT-mission.owner"
printf '%s' "$BURGER_MISSION_TOKEN" > "$OWNER"
trap 'rm -f -- "$OWNER"' EXIT
python3 "$HERE/sequence_runner.py" "$@" --robot "$ROBOT"
