# 버거 최신 통신 변경분의 PC·AMR 반영

2026-10-01 버거1·버거2에서 소스 스냅샷을 다시 가져왔습니다. 실제 로봇은 읽기만 했습니다.

## 반영 범위

- PC 원본 수집: `final_robot_ws/robot/burger1`, `robot/burger2`. 로봇별 실제 소스와 수집 manifest 갱신.
- 기존 PC/AMR 수정사항은 이전 로봇 스냅샷을 기준으로 3방향 병합. REST는 새 GraphGuard와 기존 IR+odom 정지 검증을 모두 유지.
- `communication_guard.py`를 공통 소스로 두고 각 로봇의 navigation·camera 실행 디렉터리에 배치. DDS 연결 조회를 모터/IR 제어 루프에서 분리.
- 카메라·odom·센서 수신의 최신 샘플 우선 정책 반영. 모터 제어의 작업 상태 파일 읽기는 별도 스레드로 이동.
- normal/parking 검출기 사전 준비와 영상 프로세스 재사용. 모드 변경 후 대상 보드와 촬영 시각이 맞는 새 프레임 3개로 준비 확인.
- 카메라 동적 프레임률 전환 및 warm parking 실행 반영.
- AMR Action의 대문자 M1/M2 주소, 모터 속도 상한·정지 래치·heartbeat 보호 유지. 일반 도킹 공통 제어/설정과 M1/M2 고유 주차 설정 유지.
- 최상위 공용 `host_ws/src` 및 지정 호스트 소스는 수정하지 않음.

원본 백업·병합 기록: PC `output/host_integration_sync_20261001_164214/`. 최신 로봇 원본과 다른 PC의 로컬 보완은 각 로봇의 `LOCAL_AMR_ADAPTATIONS.json`에 구분합니다. `.before_communication_*` 백업 파일은 수집 기록에 보관하되 AMR 실행 manifest에서는 제외합니다.

## 호스트 연동

지정된 `Desktop/final_251001`의 원본 Action client를 사용해 통신을 확인했습니다. [호스트 호환성 및 수정하지 않은 치명적 문제](reference_host/2026-10-01.md)를 확인하세요. 호스트의 전체 공정 제어는 해당 결함이 남아 있으므로 완료된 것으로 표시하지 않습니다.

## 검증

자동 테스트 **39회 통과**, 재실행 없음(한도 50회).

- 통신 감시·보드 전환 준비·패키징·도킹 공통 정책·REST 완료 증거 29개.
- 원본 호스트 M1/M2 client의 성공·실패 결과 4개, 기존 Action의 완료·정지/해제·취소 6개.
- ROS 테스트는 도메인 232·LOCALHOST, 모의 모터 실행기 사용. 종료 시 기존 rclpy Destroyable 경고는 일부 남음.

실물 네트워크 통합 주행, 모터·GPIO·카메라를 포함한 실제 성공 검증은 별도입니다. 실제 로봇에는 AMR 변경분을 배포하지 않았습니다.
