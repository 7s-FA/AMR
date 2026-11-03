#!/usr/bin/env bash
# ========================================================================
# 역할: 로봇 터미널용 짧은 명령 (mat/asm/park/rest 제출, mode/status/stop) → operation.py.
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
ACTION="${1:-status}"
shift || true
case "$ACTION" in
 mat|asm|park|rest)
  if [[ "${1:-}" == --dry-run ]]; then
   exec bash "$HERE/run_selected_waypoints.sh" "$ACTION" --dry-run
  fi
  exec python3 "$HERE/operation.py" submit "$ACTION" "$@" ;;
 mode|status|stop) exec python3 "$HERE/operation.py" "$ACTION" "$@" ;;
 *) echo '사용법: mission.sh mat|asm|park|rest|mode|status|stop'; exit 2 ;;
esac
