#!/usr/bin/env bash
# ========================================================================
# 역할: 로봇 전원을 켜면 관제 PC 없이 스스로 '관제 준비'를 한다 (2026-10-11 추가). 이동 명령은 보내지 않는다.
#       host_prepare.py 요청을 로봇 안에서 보낸다: 본체(모터·라이다) 시작 → 위치추정(부팅 때 주차 좌표)·Nav2 확인
#       → 모터 토크 → Action 서버 확인 → process 모드. 처음 켤 때는 Nav2 가 늦게 올라와 실패할 수 있어 몇 번 다시 시도한다.
# 실행: <로봇>-boot-prepare.service (부팅 때 한 번, systemctl --user enable 로 등록). 로그: journalctl --user -u <로봇>-boot-prepare
# 참고: 긴급정지 래치는 풀지 않는다(관제 RESTART 로만 해제). 나중에 관제 PC 가 준비를 다시 보내도 같은 절차라 문제없다.
# ========================================================================
# [공통] 위 폴더로 올라가며 runtime.env 를 찾아 실행 경로 변수(AMR_WORKSPACE, AMR_CAMERA 등)를 불러온다.
_amr_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
while [[ "$_amr_dir" != / && ! -f "$_amr_dir/runtime.env" ]]; do
  _amr_dir=$(dirname "$_amr_dir")
done
if [[ -f "$_amr_dir/runtime.env" ]]; then source "$_amr_dir/runtime.env"; fi
unset _amr_dir
set -uo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROBOT=$(basename "$(dirname "$HERE")")
source "$HERE/nav_env.bash" || exit 1
# 시도 횟수·간격은 서비스 파일의 Environment= 로 바꿀 수 있다.
ATTEMPTS=${BOOT_PREPARE_ATTEMPTS:-4}
RETRY_WAIT_S=${BOOT_PREPARE_RETRY_WAIT_S:-40}
START_DELAY_S=${BOOT_PREPARE_START_DELAY_S:-20}
BOOT=$(tr -d '-' < /proc/sys/kernel/random/boot_id | cut -c1-12)

echo "$ROBOT 부팅 관제 준비: ${START_DELAY_S}초 뒤 시작 (Wi-Fi·본체 장치 대기, 이동 명령 없음)"
sleep "$START_DELAY_S"
for (( attempt=1; attempt<=ATTEMPTS; attempt++ )); do
  echo "$ROBOT 부팅 관제 준비 ${attempt}/${ATTEMPTS}"
  # 요청 ID 는 부팅마다 달라야 한다 (같은 ID 를 다시 보내면 지난 결과를 그대로 돌려준다).
  if /usr/bin/python3 "$HERE/host_prepare.py" "$ROBOT" --request-id "boot-${BOOT}-${attempt}"; then
    echo "$ROBOT 부팅 관제 준비 완료: process 모드, 관제 명령을 받을 수 있음"
    exit 0
  fi
  if (( attempt < ATTEMPTS )); then
    echo "$ROBOT 준비 실패: ${RETRY_WAIT_S}초 뒤 다시 시도 (처음 켤 때 Nav2 가 늦게 올라올 수 있음)"
    sleep "$RETRY_WAIT_S"
  fi
done
echo "$ROBOT 부팅 관제 준비 실패: 관제 PC 에서 준비를 다시 보내거나(robot_prepare.py, 주차 위치 복구) 로봇 상태를 확인하세요." >&2
exit 1
