#!/usr/bin/env bash
# Only an exit-on-result DOCKED success arms departure for the next waypoint run.
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$HERE/../../.." && pwd)"
FLAG="$ROOT/data/burger2/departure_pending"
mkdir -p "$(dirname -- "$FLAG")"
rm -f -- "$FLAG"
if /bin/bash "$@"; then
  TEMP=$(mktemp "$FLAG.XXXXXX")
  printf 'successful_docking %s\n' "$(date --iso-8601=seconds)" > "$TEMP"
  mv -f -- "$TEMP" "$FLAG"
  echo '도킹/파킹 완료: 다음 웨이포인트에서 출차 후진·180도 회전을 한 번 실행합니다.'
else
  status=$?
  rm -f -- "$FLAG"
  exit "$status"
fi
