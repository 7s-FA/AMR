#!/usr/bin/env bash
# ========================================================================
# 역할: 관제 준비 작업자(host_prepare.py --serve) 실행. burger1-host-ready.service 의 본체.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -eo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
source "$HERE/nav_env.bash"
exec /usr/bin/python3 "$HERE/host_prepare.py" "$ROBOT" --serve
