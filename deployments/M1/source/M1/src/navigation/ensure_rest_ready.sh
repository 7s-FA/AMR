#!/usr/bin/env bash
# ========================================================================
# 역할: REST 대기 작업자(burger1-rest-ready)가 응답하는지 확인하고, 없으면 켠 뒤 최대 10초 기다린다.
# 호출: ready_parallel.py.
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
export BURGER_PREPARED_SOCKET_NAME=burger1-rest-ready.sock
if python3 "$HERE/waypoint_client.py" "$HERE/rest_forward.py" --check; then exit 0; fi
if systemctl --user is-active --quiet burger1-rest.service;then
 echo 'REST 실행 중에는 준비 프로세스를 재시작하지 않습니다.' >&2;exit 1
fi
_rest_state=$(systemctl --user show burger1-rest-ready.service -p ActiveState --value)
case "$_rest_state" in
 active|activating|reloading) ;; # Retain a running worker while its socket initializes.
 deactivating) echo 'REST 준비 프로세스 종료 중' >&2;exit 1 ;;
 *) systemctl --user start burger1-rest-ready.service ;;
esac

python3 "$HERE/waypoint_client.py" "$HERE/rest_forward.py" --wait-ready 10
