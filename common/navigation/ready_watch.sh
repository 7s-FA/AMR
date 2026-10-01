#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
bash "$HERE/manage.sh" ready || echo "준비 확인에 실패했습니다. 상태를 표시합니다. 현재 위치와 서비스 로그를 확인하세요." >&2
exec bash "$HERE/watch_ready.sh" "$@"
