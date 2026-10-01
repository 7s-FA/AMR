#!/usr/bin/env bash
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
