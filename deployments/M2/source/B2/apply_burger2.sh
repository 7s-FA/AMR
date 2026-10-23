#!/usr/bin/env bash
# ========================================================================
# 역할: 버거2를 버거3와 같은 도킹·rest 방식(IR 대신 엔코더 거리 정지)으로 바꾼다. 원본 AMR 코드는 수정하지 않는다.
#   1) docking.yaml 을 B2/config 사본으로 (make_docking_override.py 로 생성)
#   2) 도킹 상태 기계·성공 판정은 B2 사본으로 링크, docking_node·retry_ready·rest_forward 는 B2 사본 복사,
#      docking_standby·sequence_runner 는 원본 복사 (파이썬은 링크된 스크립트의 '원래 폴더'에서 모듈을 찾기 때문)
# robot.py --robot M2 configure 를 다시 하면 링크가 원래대로 돌아가므로 이 스크립트를 다시 실행한다.
# 되돌리기: bash apply_burger2.sh --revert
# ========================================================================
set -euo pipefail
B2="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AMR="$(dirname "$B2")"
CAM="$AMR/.runtime/M2/final_robot_camera"
NAV="$AMR/.runtime/M2/final_robot_ws/robot/burger2/navigation"
[[ -d "$CAM" && -d "$NAV" ]] || { echo "먼저: cd $AMR && python3 scripts/robot.py --robot M2 configure" >&2; exit 1; }
if [[ "${1:-}" == --revert ]]; then
  ln -sfn ../../../M2/config/docking.yaml "$CAM/docking.yaml"
  for f in docking_control docking_node docking_standby; do ln -sfn "../../../common/runtime/$f.py" "$CAM/$f.py"; done
  for f in terminal_evidence sequence_runner rest_forward; do ln -sfn "../../../../../../common/navigation/$f.py" "$NAV/$f.py"; done
  ln -sfn ../../../../../../M2/src/navigation/retry_ready.py "$NAV/retry_ready.py"
  ln -sfn ../../../../../../M2/src/navigation/run_selected_waypoints.sh "$NAV/run_selected_waypoints.sh"
  echo "원래 링크로 되돌림 (도킹 대기·rest 대기 재시작 필요)"; exit 0
fi
python3 "$B2/make_docking_override.py"
ln -sfn ../../M2/config/docking_board.yaml "$B2/config/docking_board.yaml"
ln -sfn ../../M2/config/parking_board.yaml "$B2/config/parking_board.yaml"
ln -sfn ../../M2/config/camera.yaml "$B2/config/camera.yaml"   # docking.yaml 의 calibration_file 을 같은 폴더에서 찾음
ln -sfn "$B2/config/docking.yaml" "$CAM/docking.yaml"
ln -sfn "$B2/docking_control_b2.py" "$CAM/docking_control.py"
cp --remove-destination "$B2/docking_node_b2.py" "$CAM/docking_node.py"
cp --remove-destination "$AMR/common/runtime/docking_standby.py" "$CAM/docking_standby.py"
ln -sfn "$B2/terminal_evidence_b2.py" "$NAV/terminal_evidence.py"
cp --remove-destination "$AMR/common/navigation/sequence_runner.py" "$NAV/sequence_runner.py"
cp --remove-destination "$B2/retry_ready_b2.py" "$NAV/retry_ready.py"
cp --remove-destination "$B2/rest_forward_b2.py" "$NAV/rest_forward.py"
cp --remove-destination "$B2/run_selected_waypoints_b2.sh" "$NAV/run_selected_waypoints.sh"   # 조립대 경유 지점 도착 방향 1.5°
echo "B2 사본: docking.yaml, docking_control.py, terminal_evidence.py (링크) / docking_node.py, retry_ready.py, rest_forward.py (B2 사본 복사) / docking_standby.py, sequence_runner.py (원본 복사)"
