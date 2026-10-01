# 최초 이관 검증 기록 — 2026-10-01

이 문서의 484개는 최초 이관 시 결과입니다. 이후 실제 로봇 변경분 동기화와 일반 도킹 공통화에 대한 최신 결과(556개)는 [동기화 검증 기록](robot_sync_2026-10-01.md)을 확인하세요.

검증 대상은 새 `/home/jh/AMR` 소스입니다. 기존 호스트 프로젝트·실제 로봇 실행 파일을 교체하거나 주행 명령을 보내지 않았습니다.

## 결과

| 검사 | 결과 |
|---|---|
| Burger.action ROSIDL 및 amr_mission 빌드 | 통과 |
| M1 프로필로 구성한 waffle_navigation C++/ROS 패키지 빌드 | 통과 |
| M2 프로필로 구성한 waffle_navigation C++/ROS 패키지 빌드 | 통과 |
| Action 입력·정지/속도 게이트·실제 Backend의 모의 Operation·패키징 | 33개 통과 |
| ROS Action 클라이언트/서버 통신, 완료·중복·정지·해제·취소·모드 거부 | 5개 통과 |
| M1 구성의 기존 Nav2 회귀 검사 | 193개 통과 |
| M2 구성의 기존 Nav2 회귀 검사 | 180개 통과 |
| M1 도킹 제어·비전 회귀 검사 | 37개 통과 |
| M2 도킹 제어·비전 회귀 검사 | 36개 통과 |

총 484개 검사가 통과했습니다. ROS 통신 검사는 실제 Action 메시지를 사용하되 모터 명령을 보내지 않는 FakeBackend로 실행했습니다. 도메인 232·localhost로 실제 로봇 도메인 40과 분리했습니다. 두 프로필의 검사 수 차이는 기존 로봇 스냅샷에 포함된 회귀 검사 차이입니다.

패키징 중 드러난 오래된 테스트 기대값은 현재 로봇 설정과 맞췄습니다. 두 로봇의 정본 설정을 모든 실행 구성에 사용하고, DWB의 정지/회전 샘플 허용 및 PositionApproach의 저속 병진 필터를 현재 구현으로 검사합니다. 테스트에 맞추기 위해 Nav2 주행 파라미터를 바꾸지 않았습니다.

## 재현

```bash
python3 -m pytest -q tests/test_action_contract.py tests/test_backend.py tests/test_packaging.py
```

로봇 실행 구성을 생성하고 ROS 패키지를 빌드한 후 해당 install/setup.bash를 source합니다. ROS 통신 검사에서는 외부 DDS 프로필·discovery server 설정을 제거하고 localhost로 제한합니다.

```bash
ROS_DOMAIN_ID=232 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 -m pytest -q tests/test_action_ros.py
python3 -m pytest -q "$HOME/amr_runtime/M2/final_robot_ws/host_ws/src/waffle_navigation/test"
```

M1도 해당 실행 구성에서 같은 주행 검사 절차를 사용합니다. C++ 검사에는 ROS Jazzy DWB 헤더와 컴파일러가 필요합니다.

## 검증 범위 밖

새 Action 서버를 통한 실제 M1/M2 주행·도킹, cmd_val별 실속도, 정지 명령 후 물리적 정지 거리, 여러 로봇을 동시에 사용하는 공정 통합은 아직 검증하지 않았습니다. 모의 Action 검사를 실제 로봇 성공 기록으로 해석하면 안 됩니다.

기존 본체 드라이버의 stack smashing / 종료 코드 -6, odom/TF 단절, 환경별 도킹 실패는 [남은 문제](known_issues.md)에 보존했습니다. 이번 저장소 이관이 해당 현장 문제의 해결 증거는 아닙니다.
