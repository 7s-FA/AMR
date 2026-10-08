# Load paths belonging to this AMR runtime (also in systemd jobs).
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
