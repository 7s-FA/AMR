# 호스트와 로봇 Action 계약

2026-10-01 첨부 표와 사용자 확인 기준. `RESTART`는 비상정지 해제만 수행합니다.

## 연결

| 항목 | M1 | M2 |
|---|---|---|
| Action Server | `/M1/data` | `/M2/data` |
| 타입 | `host_pkg/action/Burger` | 동일 |
| Action 노드 namespace | `/M1` | `/M2` |
| 내부 센서·Nav2 namespace | `burger1` | `burger2` |
| 진단 토픽 | `/M1/mission/diagnostics` | `/M2/mission/diagnostics` |
| ROS domain | 40 | 40 |

호스트 기준은 사용자가 지정한 `/home/jh/Desktop/final_251001/final_project_ws/host/src/host_pkg`입니다. 이 폴더의 `Burger.action` 필드·순서와 M1/M2 Action 주소가 AMR과 일치합니다. 원본 호스트 client로 Goal/Feedback/Result 통신을 확인했습니다. 호스트 소스는 수정하지 않았습니다.

## Goal

- `string command`: `GO_TO_MAT`, `GO_TO_ASM`, `GO_TO_REST`, `GO_TO_PARK`, `EMER_STOP`, `RESTART`
- `float32 cmd_val`: 설정 최대속도 대비 0~100%. m/s가 아닙니다. NaN/Inf/범위 밖 입력은 거부합니다. 이동 명령의 0%는 목적지에 도달할 수 없으므로 거부합니다. 제어 명령은 0%를 사용할 수 있습니다.
- 이동은 process 모드에서만 접수하며 중복 이동·정지 래치 상태의 이동은 거부합니다. 대기열은 만들지 않습니다.
- ROS Action goal UUID를 내부 command-id로 기록합니다. 이미 접수한 작업의 결과를 기다리는 것이 새로운 goal UUID로 다시 전송하는 것보다 우선입니다.

## 경로와 종단 동작

| command | 내부 경로 | 끝 동작 |
|---|---|---|
| GO_TO_MAT | mat | 일반 마커 도킹 |
| GO_TO_ASM | asm | 일반 마커 도킹 |
| GO_TO_REST | rest | 대기장소 IR 정지 |
| GO_TO_PARK | park | 로봇별 주차 마커 도킹 |
| EMER_STOP | 진행 작업 중단 | 정지 래치 유지; 신선한 odom 정지 확인 후 성공 |
| RESTART | 경로 실행 없음 | 로봇이 정지해 있고 작업 종료 후 래치만 해제 |

정지 명령은 진행 중 이동 Action과 함께 접수할 수 있습니다. 기존 이동은 성공으로 반환하지 않습니다. 로봇이 실제로 정지했다는 odom 확인이 없으면 EMER_STOP 결과도 ERROR이며, 정지 래치는 계속 유지됩니다. 이 명령은 소프트웨어 정지 요청이며 물리적 비상정지 회로를 대체하지 않습니다.

Action 취소·서버 종료·작업 실패도 정지 래치를 설정합니다. 실패 원인을 점검한 뒤 RESTART를 보내고 새 경로를 요청합니다. 서버 재기동 시 미완료 goal을 발견해도 자동 재개하지 않습니다.

## 속도

단일 모터 발행자인 motion_owner가 로봇별 설정 최대 선속도·각속도의 cmd_val%를 최종 출력 상한으로 적용합니다. 예: M2의 최대 선속도 0.066m/s, 50% 요청이면 0.033m/s 이하입니다. 원래 도킹 제어 속도가 이보다 작으면 더 느린 값이 유지됩니다. 도킹·출차·Nav2 모두 같은 출력 제한을 받습니다.

시간제어 출차는 제한된 속도에 맞춰 시간을 다시 계산합니다. 낮은 백분율은 모터 최소 구동 속도·속도 양자화 및 기존 시간 제한 때문에 이동을 완료하지 못할 수 있습니다. 낮은 값의 성공을 보장하지 않으며 실제 로봇에서 사용 가능한 범위를 검증해야 합니다. 이동 시간 제한이나 센서 watchdog을 늘려 우회하지 않았습니다.

진행 중 Action heartbeat가 1초 넘게 끊기면 motion_owner 출력은 0이 됩니다. 이 보호는 갱신이 정상인 새 Action 서버와 변경된 motion_owner를 함께 배포했을 때 적용됩니다.

## Feedback / Result

- Feedback `robot_x`, `robot_y`: 최신 AMCL map 좌표(m). 유효한 최신 위치가 없으면 가짜 0 좌표를 발행하지 않습니다.
- Feedback `robot_theta`: `FORWARD`/`BACKWARD`, odom 선속도 부호를 기준으로 하며 정지 시 마지막 방향 유지. 수치 yaw가 아닙니다.
- Feedback `message`: 원본 표의 정상 값 `IDLE`. 이 문자열만 보고 도착 완료로 판단하면 안 됩니다.
- Result `success=true`, `message=IDLE`: 경로와 종단 정지까지 확인한 성공, 또는 제어 명령의 성공.
- 실패는 `success=false`, `message=ERROR` 및 Action aborted/canceled. 상세 이유·goal_id는 진단 토픽, journal, 로봇 작업 JSON에 기록합니다.

## 지정 호스트의 남은 문제

호스트의 M1 결과 변수명 오류, M2 완료 플래그 누락, 단일 로봇 미구현, 정지 명령 미전송과 무제한 대기 때문에 전체 공정 진행은 보장할 수 없습니다. 사용자 요청으로 수정하지 않았습니다. [원본 코드 위치·영향·실행 안내](reference_host/2026-10-01.md)를 확인하세요.

## 종단 완료 근거

경로의 끝 동작은 `config/routes.yaml`의 `terminal`에서 선택합니다. 일반/주차 도킹은 컨트롤러의 `DOCKED / ir_high_and_stationary` 보고가 필요합니다. REST는 IR HIGH 후 최신 odom(0.3초 이내)으로 0.5초 연속 정지를 확인해야 성공합니다. 정지 확인은 최대 3초이며 IR 해제·odom 끊김은 실패합니다. REST 속도와 제한은 `docking.yaml`의 독립된 `rest_control`에서 관리합니다. 프로세스 종료 코드 0만으로 도착 성공을 만들지 않습니다.

[별도 테스트 호스트](../tools/test_host/README.md)도 같은 Action을 사용합니다.
