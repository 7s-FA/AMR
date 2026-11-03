#!/usr/bin/env bash
# ========================================================================
# 역할: 준비 실행 후 상태 모니터 표시 (manage.sh ready → watch_ready.sh). 로봇 터미널 수동용.
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
ROBOT=$(basename "$(dirname "$HERE")")
bash "$HERE/manage.sh" ready || echo "준비 확인에 실패했습니다. 상태를 표시합니다. 현재 위치와 서비스 로그를 확인하세요." >&2
exec bash "$HERE/watch_ready.sh" "$@"
