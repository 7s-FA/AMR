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
