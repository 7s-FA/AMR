# 2026-10-16 엔코더 거리 도킹 (IR 센서 제거) 정리

## 무엇이 바뀌었나
- **IR 센서는 쓰지 않습니다.** 도킹·주차·rest 모두 정지 판단은 엔코더(바퀴) 거리로 합니다. GPIO 코드(`ir_sensor.py`)와 `gpio_chip/gpio_pin/ir_*` 설정은 지웠습니다.
- **M1 = 버거3 (STM32 F411 + micro-ROS)**. 이름·토픽·관제 Action 은 예전 버거1 그대로(`burger1`, `/M1/data`). OpenCR 버거1 실행 경로는 없습니다.
- **M2 = 버거2 (OpenCR)**.
- 예전 로봇별 사본 폴더(`F411/`, `B2/`)와 `apply_*.sh` 는 없어졌고 모두 본 코드에 들어갔습니다. `robot.py configure` 를 다시 해도 그대로입니다.

## 도킹 순서 (`common/runtime/docking_control.py`)
1. ALIGN: 마커 보드를 보고 정렬 지점(staging)에서 좌우·각도 허용 안으로 맞추고 0.7초 유지.
2. 정렬 확정 순간 카메라~보드 거리 z0 를 기억.
3. FINAL_APPROACH: 남은 거리 = z0 − 엔코더 이동거리. 목표 `encoder_stop_distance_m` 에서 정지.
   - **버거2 (속도 방식)**: 3.5cm/s → 마지막 2cm 는 1cm/s(`encoder_slow_*`) → 미리 정지 거리만큼 일찍 정지.
   - **버거3 (펄스 이동, `move_final_enabled`)**: `/stm32/xl430_cmd "move L R 44 번호"` 한 번 → XL430 위치 모드로 이동·정지.
     바닥에서 DONE 이 안 오면 2초 뒤 토크 끄기/켜기로 위치 모드를 끝냄. STM32 `feedback` 오류면 남은 거리만 재전송(최대 2회).
   - 공통 보호: 바퀴가 1.5초 안 움직이면 `final_approach_stalled`, 전체 45초 제한(`max_total_time_s`).
4. STOPPING → 0.5초 정지 확인 → `DOCKED / encoder_distance_and_stationary`.

## rest (`common/navigation/rest_forward.py`)
- 경유 지점 4 에서 `rest_control.distance_m` 만큼 직진(버거3 40cm, 버거2 45cm) → 정지 확인 → `RESTED / encoder_distance_and_stationary`.

## 로봇별 주요 설정 (`M1|M2/config/docking.yaml`)
| 항목 | 버거3 (M1) | 버거2 (M2) |
|---|---|---|
| 재료창고 정렬 지점 | 20cm, 근거리 회전 감소 | 22cm |
| 조립대 정렬 지점 | 22cm | 20cm, 근거리 회전 감소 |
| 정지 목표(카메라~보드) mat/asm/park | 8 / 5 / 145 mm | 10.8 / 9.5 / 148.4 mm |
| 직진 방식 | 펄스 이동(move) | 속도 + 마지막 2cm 감속 |
| 카메라 각도 보정 | −1.0° | 0 |
| 먼 거리 회전 배율 / 감쇠 | 1.0 / 0.6 | 1.0 / 0.6 |
| rest 거리 | 40cm | 45cm |
- 버거3 웨이포인트 도착 허용 4cm(`M1/config/waypoints.yaml`), 버거2 조립대 경유 지점 도착 방향 1.5°(`M2/src/navigation/run_selected_waypoints.sh`).
- 실측 22mm 기준으로 맞춘 값입니다. 실측 = 카메라~보드 + 약 13~14mm (보드·차체 구조).

## 버거3 본체 (`M1/src/runtime`)
- `start_base.sh`: micro-ROS Agent docker(`f411-agent`, 921600bps) → openocd 로 STM32 리셋 → `base.launch.py`
- `base.launch.py`: 팀원 브리지 `stm32_adapter` + `cmd_throttle.py` + `wheel_odom.py` + 라이다 + `stm32_compat.py`
- `stm32_compat.py`: OpenCR 의 `sensor_state`(토크·엔코더)·`motor_power` 대체. 모터 상태 읽기 실패(-4)는 1초까지 직전 값 유지.
- 점검: `bash check_base.sh`

## 시험
- `common/runtime/test_docking_control.py`: 속도 방식 정밀도·막힘, 펄스 이동 DONE/DONE 없음/feedback 재전송, 설정 검사.
- `tests/`: `PYTHONPATH=M1/src/navigation:common/navigation:common/runtime:common python3 -m pytest -q tests`
