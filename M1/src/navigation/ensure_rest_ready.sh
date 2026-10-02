#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
export BURGER_PREPARED_SOCKET_NAME=burger1-rest-ready.sock
if python3 "$HERE/waypoint_client.py" "$HERE/rest_forward.py" --check; then exit 0; fi
if systemctl --user is-active --quiet burger1-rest.service;then
 echo 'REST 실행 중에는 준비 프로세스를 재시작하지 않습니다.' >&2;exit 1
fi
if systemctl --user is-active --quiet burger1-rest-ready.service;then
 systemctl --user restart burger1-rest-ready.service
else
 systemctl --user start burger1-rest-ready.service
fi
for ((i=0;i<100;i++));do
 if python3 "$HERE/waypoint_client.py" "$HERE/rest_forward.py" --check;then exit 0;fi
 systemctl --user is-active --quiet burger1-rest-ready.service || exit 1
 sleep .1
done
exit 1
