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
python3 - "$FILE" "$MODE" "$status" "$LOG_FILE" <<'PY'
import sys,json,time,os
from collections import deque
path,mode,status,log=sys.argv[1:];data={'mode':mode,'status':'success' if status=='0' else 'failed','exit_code':int(status),'completed_unix':time.time(),'log_file':log}
try:
 with open(log) as stream:lines=deque(stream,maxlen=40)
 for line in lines:
  try:report=json.loads(line[line.index('{'):])
  except (ValueError,TypeError):continue
  if isinstance(report,dict) and 'state' in report and 'reason' in report:
   data['controller_state']=report['state'];data['reason']=report['reason'];data['controller_report']=report
 if status!='0' and 'reason' not in data:data['reason']=''.join(lines)[-3000:].strip() or 'controller_start_or_cleanup_failed'
except OSError:
 if status!='0':data['reason']='controller_start_or_cleanup_failed'
with open(path+'.tmp','w') as f:json.dump(data,f)
os.replace(path+'.tmp',path)
PY
if (( status == 0 )) && [[ "$MODE" == dock || "$MODE" == park || "$MODE" == rest ]]; then
 temp=$(mktemp "$FLAG.XXXXXX")
 printf 'successful_%s %s\n' "$MODE" "$(date --iso-8601=seconds)" > "$temp"
 mv -- "$temp" "$FLAG"
fi
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
