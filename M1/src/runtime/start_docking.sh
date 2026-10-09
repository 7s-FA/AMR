#!/usr/bin/env bash
# ========================================================================
# 역할: 도킹 1회 실행 래퍼. 카메라 active → start_docking_engine.sh → 끝나면 카메라 idle.
# 호출: manage.sh dock|park 가 terminal_wrapper.sh 를 거쳐 실행.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
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
