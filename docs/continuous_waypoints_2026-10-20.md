# M1·M2 중간 웨이포인트 연속 통과 (2026-10-20)

## 적용 결과

- M1 `192.0.2.104`, M2 `192.0.2.102`에 적용. 최신 odom 정지 및 활성 명령 없음 확인 후 백업·교체.
- 두 로봇의 준비된 웨이포인트 작업자를 다시 불러왔으며, 읽기 전용 `--verify` 성공(`motion_sent: false`). 실제 이동 명령은 보내지 않음.
- 자재창고 `mat` 경로는 두 지점을 하나의 `NavigateThroughPoses` 목표로 전달. 중간점에서 별도의 목표 완료·정지 대기·Spin·취소를 하지 않음.
- `asm`, `park`, `rest`처럼 지점이 하나인 경로는 기존 `goToPose` 유지. 출차 후진·회전, 최종 위치·방향 검사, 도킹 순서는 유지.
- 중간점 통과 반경은 6 cm. M1 최종 이동 도착 4 cm, M2 3 cm 기준과 회전 후 6 cm 기준은 변경하지 않음.
- M2 실기에는 `B2/run_selected_waypoints_b2.sh` 보정 사본이 있음. 원본과 실행 복사본 모두 옵션을 추가했고 조립대 최종 방향 **1.5°**를 유지함. 다른 목적지 3°도 유지.

## 동작 및 복구

`--through-intermediate`는 전진·위치 도착 후 방향 정렬 모드 전용이다. 새 행동트리 `navigate_through_forward.xml`은 매 BT tick에 `RemovePassedGoals`로 지나간 지점을 제거하고, 기존 1 Hz로 `ComputePathThroughPoses`를 갱신한다. `FollowPositionForward`, `position_goal_checker`, 장애물 감시와 속도 제한은 그대로 사용한다.

Nav2 피드백의 남은 지점 수로 통과 기록을 저장한다. 실패·재시도 때는 기록된 지점을 제외하고 남은 구간만 보낸다. 마지막 지점은 Nav2 성공만으로 완료 처리하지 않고 기존 정지·XY·yaw 검사를 통과해야 한다. 피드백 이상, 위치 정보 상실, 경로 이탈, 최종 목표 지나침, 제한 시간 초과 시 기존 취소·정지 처리를 유지한다.

코너의 곡률, 장애물, 위치 추정 변화, 제어기의 진행 실패 때문에 감속·정지는 여전히 가능하다. 이번 수정은 중간 목표를 종료하고 다음 목표를 시작하던 의도적 정지 구간을 제거한다. 실제 코너 주행 품질은 다음 실물 시험에서 확인해야 한다.

## 검증

- 신규 경로 모의 시험 및 기존 위치/방향·지나침 감시 관련 시험: **46 passed**.
- 실제 설치된 Nav2 BT navigator + 모의 planner/controller, `ROS_DOMAIN_ID=149`, `LOCALHOST`: 중간점 피드백 2→1, FollowPath 한 번, 취소 0회, 최종 성공.
- 같은 격리 시험에서 경로 취소가 FollowPath 서버로 전달되고 취소 결과로 종료되는 것 확인.
- 전체 기존 시험의 실패 15건과 건너뜀 5건은 변경 전 코드에서도 동일하게 재현. 설정 파일의 옛 경로 참조 및 오래된 테스트 대역 관련이며 이번 수정으로 새 실패는 추가되지 않음.
- M1·M2 실제 `mat --dry-run`에 연속 통과 옵션 확인. M2 `asm --dry-run`에서 최종 yaw 1.5° 유지 확인.
- 두 프로필을 임시 폴더에 materialize하여 행동트리·신규 테스트·실행 옵션이 재생성되는 것 확인.

시험 예시(관제 domain 40에서는 실행하지 않음):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=149 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI ROS_DISCOVERY_SERVER ROS_STATIC_PEERS
python3 common/src/waffle_navigation/test/through_bt_smoke.py
python3 common/src/waffle_navigation/test/through_bt_smoke.py --cancel
```

## 배포·백업

- 공통 실행 코드 SHA-256: `d37f8e6be8559289524cb4612e89a6877398fde27419c9a1d84db47bb76535fd`
- 행동트리 SHA-256: `c7b64479e70006a1dfa58736e205127232b1fea1f699a3467edfafab2a602b9b`
- M1 변경 전: `/home/example_robot/through-waypoints-20261020/backup-1792459442804396056`
- M1 manifest 변경 전: `/home/example_robot/through-waypoints-20261020/runtime-backup-1792459599235749897`
- M2 변경 전: `/home/example_robot/through-waypoints-20261020/backup-1792459458004045922`
- M2 B2 보정 사본·실행 복사본·manifest 변경 전: `/home/example_robot/through-waypoints-20261020/runtime-backup-1792459586685016082`

되돌릴 때도 진행 중 명령 없음과 정지를 확인한 뒤 백업한 실행 코드·wrapper를 복원하고 웨이포인트 작업자를 다시 불러와야 한다. M2는 일반 원본뿐 아니라 B2 보정 사본과 실행 복사본도 함께 복원해야 한다. 주행·도킹·배터리 펌웨어나 속도 설정은 이번 변경 대상이 아니다.
