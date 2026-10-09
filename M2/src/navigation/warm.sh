#!/usr/bin/env bash
# ========================================================================
# 역할: 주행 전 기본 준비. 본체(base)·모터 관리자·위치추정 서비스를 켜고 check_ready.py 로 위치추정 확인.
# 호출: manage.sh, run_selected_waypoints.sh, rest.sh, run_rest.sh, ready_parallel.py, confirm_parked.sh(--force-parked).
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
CAMERA_ROOT=${AMR_CAMERA}
[[ "$ROBOT" != burger2 ]] || CAMERA_ROOT=${AMR_CAMERA}
if [[ ! $(systemctl --user show "$ROBOT-base.service" -p ActiveState --value) =~ ^(active|activating|reloading)$ ]];then
 if pgrep -u "$USER" -f '[/]turtlebot3_ros|[/]ld08_driver|[b]ase.launch.py' >/dev/null;then
  echo '수동 본체 실행을 먼저 종료하세요.' >&2;exit 1
 fi
 systemctl --user reset-failed "$ROBOT-base.service" 2>/dev/null || true
 # Prefer deployed persistent units; creating a transient unit with the same name fails.
 _service_fragment=$(systemctl --user show "$ROBOT-base.service" -p FragmentPath --value 2>/dev/null || true)
 if [[ -n "$_service_fragment" ]]; then
  systemctl --user start "$ROBOT-base.service"
 else
  systemd-run --user --collect --unit="$ROBOT-base" --property=KillSignal=SIGINT --property=TimeoutStopSec=5 --setenv=ROS_DOMAIN_ID=40 /bin/bash "$CAMERA_ROOT/start_base.sh"
 fi
 unset _service_fragment
fi
for _warm_unit in "$ROBOT-motion-owner.service" "$ROBOT-localization.service"; do
 _warm_state=$(systemctl --user show "$_warm_unit" -p ActiveState --value)
 case "$_warm_state" in active|activating|reloading) ;; deactivating) echo "$_warm_unit 종료 중" >&2;exit 1;; *) systemctl --user start "$_warm_unit";; esac
done
unset _warm_unit _warm_state

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

if [[ "${BURGER_PARALLEL_PREPARE:-}" != 1 ]]; then
# Prepare ROS connections once; no goals or motor/GPIO commands.
# Preparation failure uses the original per-route path.
bash "$HERE/ensure_waypoint_ready.sh" || echo "웨이포인트 연결 사전 준비 실패: 기존 실행 경로 유지" >&2

# REST standby retains feedback/graph connections; owns no GPIO or speed publisher.
bash "$HERE/ensure_rest_ready.sh" || echo 'REST 사전 준비 실패: 기존 실행 경로 유지' >&2

fi
