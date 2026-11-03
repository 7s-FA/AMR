# ========================================================================
# 역할: 이동 시작 전 잠금 확인 함수(mission_guard). 준비 중이거나 다른 임무가 돌고 있으면 거부한다.
#       같은 임무(토큰 일치)의 하위 단계는 통과. 사용처: manage.sh, rest.sh, run_selected_waypoints.sh, confirm_parked.sh.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
# Source before taking a robot motion lock.
mission_guard() {
 local robot="$1" owner="${XDG_RUNTIME_DIR:-/tmp}/$1-mission.owner"
 local lock="${XDG_RUNTIME_DIR:-/tmp}/$1-mission-$UID.lock"
 local preparation="${XDG_RUNTIME_DIR:-/tmp}/$1-prepare-$UID.lock"
 if [[ -n "${2:-}" ]]; then
  # Reuse the caller's inherited lock, never an unchecked bypass flag.
  if ! [[ "$2" =~ ^[0-9]+$ && "/proc/$$/fd/$2" -ef "$preparation" ]] || ! flock -n "$2"; then
   echo '준비 잠금 소유권을 확인할 수 없습니다.' >&2
   return 1
  fi
 elif ! flock -n "$preparation" true; then
  echo '로봇 준비 중입니다. 준비 완료 후 명령하세요.' >&2
  return 1
 fi
 # Closing this descriptor immediately only probes whether a mission owns it.
 if ! flock -n "$lock" true; then
  if [[ -z "${BURGER_MISSION_TOKEN:-}" || ! -f "$owner" || "$(cat "$owner")" != "$BURGER_MISSION_TOKEN" ]]; then
   echo '전체 이동 시퀀스 실행 중입니다. mission_stop으로 먼저 종료하세요.' >&2
   return 1
  fi
 fi
}
