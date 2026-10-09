# AMR 코드 안내 (2026-10-09 정리·최적화본)

이 문서는 **설명용 안내서**입니다. 코드를 처음 보는 사람이 "무엇이 우리 코드이고, 어디서부터 읽어야 하고, 오늘 무엇을 바꿨는지"를 한 번에 볼 수 있게 정리했습니다.
각 파일 맨 위에도 같은 내용(역할 / 실행 / 호출 관계)을 한글 주석으로 달았습니다.

---

## 1. 한눈에 보는 동작 흐름

```text
관제 PC (host_backend)
   │  ROS 액션 /M1/data, /M2/data  (command=GO_TO_MAT…, cmd_val=속도%)
   ▼
action_server.py ──► backend.py ──► operation.py (접수·모드·중복·이동중 검사)
                                        │ systemd-run <로봇>-mission
                                        ▼
                                  mission_entry.sh ──► sequence_runner.py
                                                         │
                       ┌─────────────────────────────────┴─────────────────────┐
                       ▼ ① 경로 주행                                           ▼ ② 종단 동작
          run_selected_waypoints.sh                                  manage.sh dock|park  /  rest.sh start
          → station_routes.py (경로 파일)                             → terminal_wrapper.sh
          → run_waypoints.sh → waypoint_client.py                     → start_docking.sh → docking_warm_client
          → waypoint_worker.py (대기 작업자)                            → docking_standby.py → docking_node.py
          → nav2_waypoints.py (출차·주행·도착 판정)                      (docking_control.py 계산, IR 정지)
                                                                      rest: run_rest.sh → rest_ready_worker → rest_forward.py
                       └────────────── 결과: data/<로봇>/commands/<id>.json ──────┘
                                        ▲
action_server.py 가 0.2초마다 결과 파일을 읽어 관제에 IDLE / ERROR 로 돌려줌
```

모터 명령은 항상 **motion_owner.py 하나**만 내보냅니다.
- 경로 주행 중에는 `nav` 모드(Nav2 출력)만, 도킹·rest 중에는 `direct` 모드(직접 제어)만 통과시킵니다.
- 모드 전환은 `set_mode.sh → motion_client.py → motion_mode.py` 경로로 요청합니다.
- 긴급정지·속도%는 `data/<로봇>/action_gate.json` 파일로 motion_owner 가 매 출력마다 적용합니다.

### 상시 실행되는 서비스 (로봇 1대당)

| 서비스 | 실행 파일 | 하는 일 |
|---|---|---|
| burgerN-base | start_base.sh → base.launch.py | 모터·odom·라이다 드라이버 (TurtleBot3) |
| burgerN-camera | start_camera.sh → native_camera | 카메라 → 공유메모리 프레임 |
| burgerN-localization | launch_localization.sh | map_server + AMCL (위치추정) |
| burgerN-motion-owner | launch_owner.sh → motion_owner.py | 모터 명령 관리자 |
| burgerN-nav2 | launch_nav2.sh → burgerN_navigation.launch.py | Nav2 (계획·제어·회전) |
| burgerN-nav-control | nav_control_service.sh → motion_mode.py | 주행 모드 전환기 |
| burgerN-waypoint-ready | start_waypoint_worker.sh → waypoint_worker.py | 경로 주행 대기 작업자 |
| burgerN-rest-ready | start_rest_ready.sh → rest_ready_worker.py | REST 대기 작업자 |
| burgerN-docking-ready | start_docking_standby.sh → docking_standby.py | 도킹 대기 작업자(카메라 검출) |
| burgerN-host-ready | start_host_ready.sh → host_prepare.py | 관제 '로봇 준비' 요청 처리 |
| mN-action | .runtime/MN/start_action.sh → action_server | 관제 액션 서버 |

---

## 2. 우리가 직접 만든 코드 vs 원본(Nav2·TurtleBot3 등)을 가져온 코드

### 2-1. 우리 코드 (주석 대상, 설명할 부분)

| 위치 | 내용 |
|---|---|
| `common/src/amr_mission/` | 관제 액션 서버(action_server.py), 연결부(backend.py) |
| `common/navigation/` | 임무 접수·순서 실행·모드 전환·모터 관리·준비·재시도·REST 등 **운용 로직 전부** |
| `common/runtime/` | 도킹(제어 상태기계·노드·대기 작업자·영상처리), 카메라 연결, DDS 네트워크 설정, 카메라 캡처(native_camera.cpp) |
| `common/docking_vision/` | 마커 보드 자세 추정, 보정 도구 (vision.py 의 기본 알고리즘은 PinkLAB 강의 코드 기반) |
| `common/network/`, `common/runtime_env/` | Nav2 DDS 설정, ROS 환경 캐시 |
| `common/src/waffle_navigation/scripts/nav2_waypoints.py` | 경로 주행 본체 (Nav2 의 BasicNavigator 를 상속해 출차·도착 판정·재시도를 직접 구현) |
| `common/src/waffle_navigation/src/*.cpp`, `include/*.hpp` | **우리가 만든 Nav2 플러그인**: PrecisionSpin·DepartureSpin(회전), PositionApproachCritic·PrecisionPoseCritic(DWB 평가) |
| `common/src/waffle_navigation/launch/burger1/2_navigation.launch.py`, `config/arrival_tuning.py`, `scripts/save_start_pose.py` | Nav2 실행 래퍼, 도착 조정값 병합, 초기 위치 저장 도구 |
| `M1/`, `M2/` | 로봇별 설정(config)·실행 스크립트(src)·환경(runtime_env) |
| `scripts/` | 설치·빌드·실행 폴더 생성(materialize.py)·진입점(robot.py) |
| `systemd/`, `tools/test_host/` | 서비스 정의, 관제 없이 시험하는 액션 클라이언트 |

### 2-2. 원본을 가져와 일부만 바꾼 것 (줄 단위 주석 안 함, 바꾼 곳만 아래에 기록)

| 파일 | 원본 | 우리가 바꾼 점 |
|---|---|---|
| `launch/lean_bringup_launch.py` | nav2_bringup `bringup_launch.py` | 아래 navigation launch 를 우리 것으로 교체, Nav2 컨테이너가 죽으면 launch 전체 종료(서비스가 '살아있는 척' 안 하게) |
| `launch/lean_navigation_launch.py` | nav2_bringup `navigation_launch.py` | 안 쓰는 서버 제거(route_server, waypoint_follower, docking_server), bond 토픽 분리(nav_bond), bond_timeout 4→20초(Wi-Fi 발견 지연), 컨테이너 내부 통신(intra-process) 사용 |
| `behavior_trees/navigate_*.xml` | nav2_bt_navigator `navigate_w_replanning_time.xml` | 컨트롤러를 전진 전용/후진 전용/정밀 전진으로 고정, 정밀 도착 판정기 지정 |
| `M1/config/nav2.yaml`, `M2/config/nav2.yaml`, `config/nav2_burger_params.yaml` | turtlebot3_navigation2 `burger.yaml` | 로봇 접두어 좌표계, 우리 플러그인 등록·튜닝값(속도·공차·코스트맵), AMCL 초기 위치. 값 옆에 날짜 주석이 있는 줄이 우리가 바꾼 값 |
| `patches/encoder/` | turtlebot3_node `joint_state.cpp/.hpp` | 엔코더 값 튐 필터 (보관용, 자동 적용 안 함. patches/encoder/README.md) |
| `M*/src/runtime/setup_camera.sh` 가 받는 libcamera·camera_ros | 라즈베리파이 libcamera, camera_ros | 버전 고정 설치만 함 (코드 수정 없음) |

---

## 3. 2026-10-09 변경 내역

### 3-1. 지운 파일 (어디서도 쓰이지 않음을 확인)

| 지운 파일 | 이유 |
|---|---|
| `tools/legacy_camera/` | 예전 ROS 카메라 노드. native_camera 로 대체됨 |
| `scripts/start_burger1.sh`, `start_burger2.sh` | `robot.py host-ready` 를 부르기만 하던 래퍼, 사용처 없음 |
| `common/navigation/retry_ready.py` | M1/M2 각자 사본을 씀. 공통 사본은 배포되지 않음 |
| `common/runtime/communication_guard.py`, `data_flow.py` | `common/navigation/` 의 같은 파일과 완전히 동일 → 하나로 합침 (카메라 쪽도 같은 원본을 링크) |
| `launch/map_building.launch.py`, `map_view.launch.py`, `config/mapper_params.yaml`, `rviz/map_building.rviz` | 지도 작성용. 지도는 이미 완성되어 운용에 안 씀 (필요하면 git·백업에서 복구) |
| `launch/navigation2.launch.py` | PC 용 RViz 포함 launch, 사용처 없음 |
| `scripts/normalize_scan.py`, `config/nav2_params.yaml` | LDS-03 라이다용. 두 로봇 모두 LDS-02 |
| `config/waypoints.yaml` (waffle_navigation 안) | 배포되지 않는 옛 좌표 사본. 실제는 `M1/M2/config/waypoints.yaml` |
| `common/navigation/start_nav2.sh`, `run_waypoints_record.sh`, `performance_probe.py` | 호출하는 곳 없음 |
| `M*/src/navigation/operator_aliases.bash`, `performance_topics.cpp`, `M2/.../motion_trace.py` | 수동 별칭·성능 측정 도구, 사용처 없음 |
| `M*/src/runtime/start_ir.sh`, `M*/runtime_env/burgerN_remote.bash` | 사용처 없음 / 내용 없는 호환 파일 |
| `common/docking_vision/viewer.py` | PC 화면용 뷰어(burger2 고정), 로봇에서 안 씀 |
| 코드 안 죽은 부분 | `FreshSequence` 클래스, `backup_progress()`·`_departure_pose()`(엔코더 출차로 바뀐 뒤 안 씀), 안 읽는 카운터, 안 쓰는 import |

두 로봇의 AMR 폴더도 **하나로 통일**했습니다. 각자 자기 프로필은 최신이었지만, 상대 프로필 사본(버거1의 M2, 버거2의 M1)이 옛날 것이었습니다.

### 3-2. 부하 줄이기 (동작 로직은 그대로)

| 프로그램 | 원인 | 바꾼 것 | PC 측정 (같은 센서 부하, 한 코어 %) |
|---|---|---|---|
| action_server | 파이썬 멀티스레드 실행기가 odom(30Hz) 하나에도 쉬지 않고 돎 | odom·위치 수신만 별도 노드 + 싱글스레드 실행기로 분리 | 57.5 → **3.3** |
| motion_mode | 대기 중에도 TF(54Hz) 수신, 0.01초마다 깨어남 | TF 는 주행 전환 때만 구독(30초 미사용 시 해제), 0.05초 주기 | 11.1 → **2.1** |
| waypoint_worker | 대기 중에도 odom·엔코더·라이다·TF 수신 | 30초 미사용 시 구독 해제, 요청 오면 재연결(0.1초) 후 최신 값 확인 | 8.9 → **2.4** (대기 30초 후) |
| motion_owner | 대기 중에도 0 속도를 50Hz 로 발행 + 매번 래치 파일 읽기 | 대기 중 0.5초마다 1번, 0 속도는 파일 안 읽음 (주행 중은 그대로 50Hz) | 4.3 → 4.1 (로봇에선 turtlebot3_ros 부하도 같이 줄어듦) |
| docking_standby / 영상 작업자 | 0.01초 대기 루프, 대기 중 0.005초마다 새 프레임 확인 | 0.05초 루프, 대기 중 프레임 확인 0.05초 (도킹 중은 그대로) | – |
| backend.result / sequence_runner | 주행·도킹 대기 중 systemctl 을 초당 5~7번 실행 | 결과 파일은 그대로 자주, 서비스 확인만 1초에 한 번 | – |

로봇 실측(버거1, 대기 중, 변경 전): action_server 23%, waypoint_worker 22%, motion_mode 21%, motion_owner 9%, docking_standby 7%, rest_ready 5%, turtlebot3_ros 10%.

### 3-3. 고친 버그

| 파일 | 문제 | 수정 |
|---|---|---|
| `common/navigation/motion_client.py` | 모드 전환기(nav-control) 서비스가 꺼져 있을 때 요청하면, 등록된 서비스 파일이 있는데도 같은 이름의 임시 서비스를 만들려다 'already loaded' 오류로 준비가 실패 | 등록된 서비스 파일이 있으면 `systemctl start` 로 켬 (warm.sh·ensure_camera.sh 와 같은 방식) |

### 3-4. 검증

- 로봇 코드 전체 테스트(M1·M2 각각 약 390개)를 PC 에서 변경 전·후로 실행 → **새로 생긴 실패 0개**.
  남은 실패는 변경 전부터 있던 것(도킹 조정값 변경과 테스트 기대값 차이 등)입니다.
- 실제 주행·도킹은 로봇 재시작 후 확인이 필요합니다.

### 3-5. 주의 (적용 후)

- 웨이포인트·도킹·REST 대기 작업자는 **코드 파일이 바뀌면 재시작 전까지 요청을 거부**합니다(안전장치). 적용 후에는 로봇 서비스를 다시 켜야 합니다.
- 원래 파일은 각 로봇 `~/backups/amr_before_cleanup_20261009.tar.gz` 에 있습니다.
