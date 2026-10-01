#!/usr/bin/env bash
# A real controller success is required; persist the terminal result atomically.
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$HERE/../../.." && pwd)"
ROBOT=$(basename "$(dirname "$HERE")")
MODE="$1"; shift
FILE="$ROOT/data/$ROBOT/terminal_result.json"
FLAG="$ROOT/data/$ROBOT/departure_pending"
mkdir -p "$(dirname -- "$FILE")"
LOG_DIR="$ROOT/data/$ROBOT/terminal_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_$$.log"
rm -f -- "$FILE"
# Fresh attempts invalidate a previous departure claim.
if [[ "$MODE" == dock || "$MODE" == park || "$MODE" == rest ]]; then rm -f -- "$FLAG"; fi
# Always report early startup failures, including control ownership errors.
finish() {
 status=$?
 trap - EXIT
python3 "$HERE/terminal_evidence.py" "$FILE" "$MODE" "$status" "$LOG_FILE"
report_status=$?
if (( status == 0 && report_status != 0 )); then status=1; fi
if (( status == 0 )) && [[ "$MODE" == dock || "$MODE" == park || "$MODE" == rest ]]; then
 temp=$(mktemp "$FLAG.XXXXXX")
 printf 'successful_%s %s\n' "$MODE" "$(date --iso-8601=seconds)" > "$temp"
 mv -- "$temp" "$FLAG"
fi
exit "$status"
}
trap finish EXIT
bash "$HERE/set_mode.sh" direct || exit 1
printf '%s\n' "$$" > "$ROOT/data/$ROBOT/terminal_started"
status=0
/bin/bash "$@" 2>&1 | tee "$LOG_FILE"
PIPE_RESULTS=("${PIPESTATUS[@]}")
status=${PIPE_RESULTS[0]}
if (( status == 0 && PIPE_RESULTS[1] != 0 )); then status=${PIPE_RESULTS[1]}; fi
bash "$HERE/set_mode.sh" idle || status=1
exit "$status"
