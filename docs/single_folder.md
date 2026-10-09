# 같은 AMR 폴더를 버거1·버거2에서 사용

이 문서는 2026-10-06 로봇 실행 폴더 백업을 이식한 작업본의 현재 사용법입니다.
Git commit/push와 실제 로봇 서비스 전환은 수행하지 않았습니다.

## 파일 구조

두 로봇에 **같은 AMR 폴더 전체**를 넣고 M1/M2만 선택합니다. 로봇마다 소스를 따로 복사하거나 합치지 않습니다.
기존 Git과 같은 최상위 소스 디렉터리를 유지합니다. 별도 최상위 `amr` 파일이나 `local_robot.py`는 사용하지 않고,
기존 `scripts/robot.py`에 `--robot M1/M2` 선택을 추가했습니다. 기존 `--runtime 경로` 명령도 사용할 수 있습니다.
공통 ROS 패키지는 `common/src`, 공통 도킹·카메라는 `common/runtime`, 좌표·튜닝은 각 `M1/config`, `M2/config`가 원본입니다.
중첩 `overrides`와 manifest에서 사용하지 않는 이전 이식 복사본은 제거했습니다.

```text
AMR/
  common/                     # 공통 소스와 ROS 패키지
  M1/, M2/                    # 로봇별 설정 및 전용 실행 파일
  interfaces/                 # Burger Action
  systemd/                    # 서비스 템플릿
  scripts/                    # 기존 robot.py·materialize.py 진입점, 빌드·진단
  .runtime/M1 또는 M2/        # 해당 로봇에서 생성, Git 제외
    final_robot_ws/           # 소스 링크, ROS 빌드·설치, 운행 데이터
    final_robot_camera*/      # 소스 링크, 카메라 의존성·실행 파일
    units/                    # 이 AMR 위치를 참조하는 서비스 파일
```

주행·도킹 소스와 설정은 저장소 파일에 연결됩니다. `.runtime` 아래 소스 링크를 편집해도 실제 Git 소스가 변경됩니다.
빌드·로그·상태·DDS 생성 파일은 `.runtime`에만 저장됩니다. 서비스 등록 파일만 `~/.config/systemd/user`에 설치되며,
실행 경로는 AMR 내부를 가리킵니다. `/home/ubuntu/final_robot_ws`나 별도의 카메라 프로젝트는 필요하지 않습니다.

## 최초 설치

대상은 Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy입니다. OS·ROS·드라이버까지 폴더 복사로 설치되는 것은 아닙니다.
기존 로봇에 있는 ROS Jazzy, Nav2, TurtleBot3/OpenCR, LDS-02 드라이버를 사용합니다.
시스템에 드라이버가 설치돼 있으면 `~/turtlebot3_ws`는 필요하지 않고, 이 overlay가 있으면 우선 사용합니다.
`colcon`, C++ 컴파일러, Python yaml/OpenCV/gpiod, GPIO·시리얼·카메라 접근 권한도 필요합니다.
새 OS에서는 별도의 ROS/하드웨어 초기 설치가 필요하며 `doctor`가 주요 누락 항목을 표시합니다.

복사할 때 `.runtime`, `.validation`과 이전 빌드 결과는 제외하고, 저장소 소스와 `.git`을 보존하세요.
다른 로봇의 운행 상태·완료 기록·정지 래치를 복사해 사용하지 않습니다.
경로에는 공백을 넣지 마세요. 아래는 버거1 예시이며 버거2는 모든 `M1`을 `M2`로 바꿉니다.

```bash
cd ~/AMR
python3 scripts/robot.py --robot M1 configure
python3 scripts/robot.py --robot M1 build
python3 scripts/robot.py --robot M1 camera-build
```

`camera-build`는 처음에 sudo 의존성 설치와 고정된 Raspberry Pi libcamera 소스 다운로드·빌드를 수행합니다.
설치 위치는 `.runtime/M1/final_robot_camera_burger1/install`입니다. 인터넷과 빌드 시간이 필요합니다.
이후에는 설치된 libcamera로 `native_camera.cpp`를 빌드합니다. 다른 CPU의 실행 파일을 재사용하지 않습니다.

기존 서비스가 없는 새 설치에서는 다음 한 줄도 가능합니다. 서비스는 등록만 하며 시작하지 않습니다.

```bash
python3 scripts/robot.py --robot M1 setup
```

## 기존 서비스에서 전환

임무가 완료되고 로봇이 정지한 상태에서 기존 관제·로봇 서비스를 종료한 뒤 전환합니다.
실행 중인 서비스를 자동 중단하거나 기존 경로를 무조건 덮어쓰지 않습니다.

```bash
python3 scripts/robot.py --robot M1 install-services --replace
python3 scripts/robot.py --robot M1 doctor
```

다른 내용의 기존 서비스 파일은 `.runtime/M1/service_backups/날짜/`에 보존합니다.
서비스 또는 주요 로봇 프로세스가 실행 중이면 교체를 거부합니다.
`doctor`는 설치 검사이며 실제 센서·모터 정상 여부나 도킹 성공을 보장하지 않습니다.

## 실제 준비와 운용

```bash
# 현재 위치를 유지하며 준비. 센서, Nav2, 토크, 정지, Action RESTART를 확인한 뒤 process 모드.
python3 scripts/robot.py --robot M1 host-ready
python3 scripts/robot.py --robot M1 status
```

로봇이 **자기 지정 주차 위치와 방향에 실제로 정지해 있을 때만** 아래 명령을 사용합니다.

```bash
python3 scripts/robot.py --robot M1 host-ready --parked
```

`--parked`는 초기 위치를 적용합니다. 중간 위치에서는 사용하지 말고 RViz에서 현재 자세를 먼저 지정합니다.
`host-ready`는 준비·토크 활성화·RESTART를 수행하지만 이동 목적지 Goal을 보내지 않습니다.
실패하면 원인을 해결하고 재실행합니다. 화면에 서비스 active만 보인다고 준비 성공으로 판단하지 않습니다.

로컬 시험은 진행 중 작업 종료 및 Action 서비스 종료 후 개별 모드로 전환합니다.
정지 래치는 상태 파일을 편집해 우회하지 않습니다. `stop`은 로컬 중단이며 Host의 `EMER_STOP`과 구분합니다.

```bash
python3 scripts/robot.py --robot M1 mode individual
python3 scripts/robot.py --robot M1 ready
# 아래는 실제 이동 명령입니다. 준비 완료·작업 공간 확인 후 하나씩 실행합니다.
python3 scripts/robot.py --robot M1 mat
python3 scripts/robot.py --robot M1 status
python3 scripts/robot.py --robot M1 stop
```

## 수정·재빌드·이동

Git은 AMR에서 사용합니다. 이 작업본은 아직 업로드하지 않습니다.
순수 Python/셸 변경은 원본 소스에 반영되지만, 실행 중 worker가 코드를 메모리에 보관하므로
정지 후 관련 서비스를 재시작해야 합니다. C++·Action 정의·설치 구성 변경은 `build`가 필요합니다.
새 파일을 추가하면 해당 `runtime_manifest.json`의 source/target도 추가하고 새 런타임을 구성합니다.
해시 검증이 필요한 복사형 export에는 `python3 scripts/update_manifest.py`로 해시를 갱신합니다.
기본 연결형 개발 런타임은 로컬 소스 편집을 허용합니다.

AMR 경로를 옮겼거나 다른 머신의 `.runtime`까지 복사했다면, 정지 상태에서 아래 순서로 재구성합니다.

```bash
python3 scripts/robot.py --robot M1 configure --archive-runtime
python3 scripts/robot.py --robot M1 build
python3 scripts/robot.py --robot M1 camera-build
python3 scripts/robot.py --robot M1 install-services --replace
python3 scripts/robot.py --robot M1 doctor
```

이전 런타임은 `.runtime/M1_saved_날짜`에 남습니다. 새 런타임에는 이전 운행 상태를 자동 복원하지 않습니다.

## 이식 범위와 검증 경계

- 버거1의 단계별 최대 3회 재시도, 정지·odom/TF·토크·제어권 확인, 출차/완료 구간 기록을 양쪽에 적용.
- waypoint 사전 연결 worker와 REST worker를 양쪽에 적용.
- M1/M2의 좌표·초기 자세·주차 마커·ROS namespace와 현재 백업의 설정값은 각각 유지.
- Host Action 서버와 backend는 공통 소스이며 M1 전용 복구 분기를 M2에도 적용.
- 패키지 빌드와 로컬 테스트는 실기기 주행·카메라/GPIO·모터 검증을 대체하지 않음.
- 현재 ROS 드라이버와 하드웨어 권한을 사용하므로, 아무 OS에서나 폴더만 복사해 무설치 실행하는 배포본은 아님.

### 이번 백업 작업의 확인 결과

- PC에서 M1/M2 각각 `host_pkg`, `waffle_navigation`, `amr_mission` 빌드 성공.
- 최상위 `tests`: 85 passed, 2 skipped. 경로 독립성, 프로필 분리, 재시도 및 설치 링크 복구 포함.
- M1/M2 각각 `mat --dry-run` 성공, 각자의 좌표 출력 확인. 실제 이동 명령은 보내지 않음.
- 공통 카메라 모듈 로딩 성공. 이전 이식에서 빠졌던 camera_ipc·video_http·data_flow 및 M2 docking_recorder를 명세에 포함.
- 구조 정리 전 실제 사용하던 M1/M2 설정 파일과 바이트 비교하여 좌표·초기 자세·튜닝 보존 확인.
- Git 원본과 불필요하게 달랐던 파일 권한은 복원. 소스 링크로 설치되는 ROS 실행 파일에는 실행 권한 유지.
- 백업에 포함된 전체 레거시 테스트가 통과한 것은 아님. 일부 카메라 테스트의 namespace/ROS 초기화,
  이전 속도 기대값, navigation mock 및 재시도 정렬 기대값은 현재 소스와 맞지 않아 추가 정리가 필요함.
- ARM용 카메라 빌드, 두 로봇의 서비스 전환 및 실주행은 아직 검증하지 않음.
- 기존 로컬 체크아웃과 로봇은 변경하지 않았고 Git commit/push도 하지 않음.
