#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
CAMERA_ROOT=/home/ubuntu/final_robot_camera
[[ "$ROBOT" != burger1 ]] || CAMERA_ROOT=/home/ubuntu/final_robot_camera_burger1
if ! systemctl --user is-active --quiet "$ROBOT-base.service";then
 if pgrep -u "$USER" -f '[/]turtlebot3_ros|[/]ld08_driver|[b]ase.launch.py' >/dev/null;then
  echo '수동 본체 실행을 먼저 종료하세요.' >&2;exit 1
 fi
 systemctl --user reset-failed "$ROBOT-base.service" 2>/dev/null || true
 systemd-run --user --collect --unit="$ROBOT-base" --property=KillSignal=SIGINT --property=TimeoutStopSec=5 --setenv=ROS_DOMAIN_ID=40 /bin/bash "$CAMERA_ROOT/start_base.sh"
fi
systemctl --user start "$ROBOT-motion-owner.service" "$ROBOT-localization.service"

exec 7>"${XDG_RUNTIME_DIR:-/tmp}/$ROBOT-localization-init-$UID.lock"
flock -w 30 7
# Keep preparation arguments separate from the caller's waypoint options.
# Capture them before sourcing ROS setup, which may reset positional arguments.
WARM_READY_ARGS=()
case "${1:-}" in
 "") ;;
 --force-parked) WARM_READY_ARGS=(--force-parked) ;;
 *) echo "알 수 없는 준비 옵션" >&2;exit 2 ;;
esac
source "$HERE/nav_env.bash"
python3 "$HERE/check_ready.py" burger2 "${WARM_READY_ARGS[@]}"
unset WARM_READY_ARGS
flock -u 7
exec 7>&-

# REST standby retains feedback/graph connections; no idle GPIO or motor publisher.
bash "$HERE/ensure_rest_ready.sh" || echo 'REST 사전 준비 실패: 기존 실행 경로 유지' >&2
