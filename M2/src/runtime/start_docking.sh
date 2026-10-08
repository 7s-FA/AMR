#!/usr/bin/env bash
# Load paths belonging to this AMR runtime (also in systemd jobs).
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
# Readiness/rate change only. The engine still owns all control/safety checks.
bash "$HERE/camera_profile.sh" active
cleanup_camera() { status=$?;trap - EXIT;bash "$HERE/camera_profile.sh" idle || true;exit "$status"; }
trap cleanup_camera EXIT
bash "$HERE/start_docking_engine.sh" "$@"
