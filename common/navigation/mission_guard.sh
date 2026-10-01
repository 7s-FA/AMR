# Source before taking a robot motion lock.
mission_guard() {
 local robot="$1" owner="${XDG_RUNTIME_DIR:-/tmp}/$1-mission.owner"
 local lock="${XDG_RUNTIME_DIR:-/tmp}/$1-mission-$UID.lock"
 # Closing this descriptor immediately only probes whether a mission owns it.
 if ! flock -n "$lock" true; then
  if [[ -z "${BURGER_MISSION_TOKEN:-}" || ! -f "$owner" || "$(cat "$owner")" != "$BURGER_MISSION_TOKEN" ]]; then
   echo '전체 이동 시퀀스 실행 중입니다. mission_stop으로 먼저 종료하세요.' >&2
   return 1
  fi
 fi
}
