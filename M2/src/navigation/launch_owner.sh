#!/usr/bin/env bash
# ========================================================================
# 역할: 모터 명령 관리자(motion_owner.py) 실행. burger2-motion-owner.service 의 본체.
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
source "$HERE/../../../handoff/pc_burger2_env.bash"
source "$HERE/../../common/nav2_network.bash"
configure_nav2_network burger2
exec python3 "$HERE/motion_owner.py" --robot burger2
