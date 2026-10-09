#!/usr/bin/env bash
# ========================================================================
# 역할: 현재 정지 위치를 Nav2 설정의 AMCL 초기 위치로 저장 (수동 도구, save_start_pose.py).
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Robot only: save into the onboard params file used by the Nav2 service.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../../handoff/pc_burger2_env.bash"
source "$SCRIPT_DIR/../../common/nav2_network.bash"
configure_nav2_network burger2 "$@"
exec ros2 run waffle_navigation save_start_pose --params-file "$BURGER2_NAV_PARAMS" \
  --namespace burger2 "$@" --ros-args -r /tf:=tf -r /tf_static:=tf_static
