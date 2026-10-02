# 2026-10-02 로봇 현장 코드 동기화

버거1·버거2의 현재 배포 파일을 이전 배포(`8b46660`)와 비교해 PC의 AMR 소스 및 `final_robot_ws/robot/burger1`, `robot/burger2` 미러에 반영했습니다. 초기화 상태, 정지 잠금, 위치, 로그 등 실행 데이터는 복사하지 않았습니다.

## 반영 범위

- 두 로봇: 현장 Nav2 속도·도착 허용오차·way2 좌표, 도킹/REST 속도, 출차 기본 속도 0.045 m/s를 보존했습니다.
- 두 로봇: REST 연결을 미리 준비하는 대기 프로세스와 서비스, 실행 연결 재사용을 반영했습니다. 대기 중에는 GPIO나 속도 발행자를 소유하지 않습니다.
- M1: 웨이포인트 준비 프로세스 및 클라이언트, Nav2 취소 연결 재사용을 프로필 전용으로 반영했습니다. M2에는 M1 전용 구현을 덮어쓰지 않았습니다.
- M1의 최종 도킹 제한 시간 30초, M2의 12초 및 각 로봇의 주차 속도·좌표·카메라 보정 차이를 유지했습니다.
- 실행 파일과 신규 서비스의 manifest 목록 및 SHA-256을 갱신했습니다. 테스트의 예전 속도 기대값은 현장 설정값으로 갱신했습니다.
- M1에 있던 공통 Action `feedback()`의 pose logger/print 두 줄을 M2에도 배포하고 `amr_mission`을 재빌드했습니다. 실제 import 경로의 SHA-256이 PC 공통 소스와 일치합니다. M2 Action 서비스는 기존 inactive 상태로 유지했습니다.

## 검증

- 공통/호스트/패키징 테스트: 106개 통과. 종료 시 일부 rclpy Destroyable 경고가 남았으며 테스트 실패는 없었습니다.
- 각 프로필을 별도 경로에 materialize하고 `host_pkg`, `waffle_navigation`, `amr_mission` 3개 패키지 빌드 성공.
- 빌드된 M1 웨이포인트 테스트 197개, M2 182개 통과. 번호별 `--dry-run`도 포함합니다.
- 신규 테스트: 잘못된 namespace 및 오래된 요청 거부, 이미 보낸 요청의 중복 cold fallback 금지, pending 서비스 요청 정리, 배포 의존 파일 포함 여부.
- 동기화 종료 시 로봇의 스냅샷 대상 파일과 Action 파일을 재조회해 M1 145개, M2 139개 해시를 확인했습니다.

실제 주행, 토크 변경, 정지 잠금 해제, 초기 위치 재설정은 수행하지 않았습니다. 기존 `pose: None`/오래된 AMCL Feedback 및 본체·Nav2 종료 문제의 해결을 뜻하지 않습니다. 전원 투입 후 준비는 [호스트 시작 절차](host_startup_2026-10-02.md)를 따릅니다.

PC 원본 스냅샷과 빌드 검증 디렉터리: `/home/jh/final_robot_ws/output/amr_sync_20261002/`. M2 이전 Action 파일: `/home/ubuntu/final_robot_ws/host_ws/src/amr_mission/amr_mission/action_server.py.before-amr-sync-20261002`.
