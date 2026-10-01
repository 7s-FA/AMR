#!/usr/bin/env bash
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
