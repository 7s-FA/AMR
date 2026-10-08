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
export BURGER_PREPARED_SOCKET_NAME=burger2-rest-ready.sock
if python3 "$HERE/waypoint_client.py" "$HERE/rest_forward.py" --check; then exit 0; fi
if systemctl --user is-active --quiet burger2-rest.service;then
 echo 'REST 실행 중에는 준비 프로세스를 재시작하지 않습니다.' >&2;exit 1
fi
_rest_state=$(systemctl --user show burger2-rest-ready.service -p ActiveState --value)
case "$_rest_state" in
 active|activating|reloading) ;; # Retain a running worker while its socket initializes.
 deactivating) echo 'REST 준비 프로세스 종료 중' >&2;exit 1 ;;
 *) systemctl --user start burger2-rest-ready.service ;;
esac

python3 "$HERE/waypoint_client.py" "$HERE/rest_forward.py" --wait-ready 10
