# AMR — M1 / M2 onboard navigation and docking

M1(기존 Burger1), M2(기존 Burger2)의 **로봇 내부 실행 코드**입니다. 작업 브랜치는 `jh`입니다. 호스트의 웹·공정 스케줄러·Action Client는 [host_pc](https://github.com/7s-FA/host_pc)에서 관리합니다. 기존 F1(와플), M3 영역은 보존합니다.

## 로봇 디렉터리

기존 저장소의 M1/M2/M3/F1 구성을 유지합니다. 각 로봇의 `image/`는 카메라·이미지 처리 참고 자료, `src/`는 ROS 2 노드와 로봇 제어 소스, `map/`은 Nav2·SLAM 지도용입니다. 이번 작업은 M1/M2를 채우며 M3/F1의 기존 파일은 그대로 둡니다.

## 구성

- `common/navigation/`: 작업 접수, 제어권, 경로 실행, 비상정지·속도 제한
- `common/runtime/`, `common/docking_vision/`: 공통 도킹 제어·마커 검출
- `common/src/waffle_navigation/`: 검증된 ROS 주행 패키지. 첫 이관에서는 패키지 이름 유지
- `common/src/amr_mission/`: 새 로봇 ROS Action Server
- `interfaces/host_pkg/`: 첨부 규약에 따른 공통 `Burger.action`
- `M1/`, `M2/`: 각 로봇의 설정·지도·하드웨어별 코드·실행 파일 매핑
- `scripts/`: 로봇 실행 디렉터리 구성·빌드·서비스 등록·운용
- `systemd/`: 기존 로봇 서비스 정의. 생성 단계에서 실행 경로 변환
- `patches/encoder/`: 이전 엔코더 보호 수정의 소스·패치·검사
- `tests/`: 새 통신·정지·속도 제한·패키징 검사
- `docs/`: 운용법, 호스트 계약, 검증과 미해결 문제

같은 파일은 한 곳에서 관리하고 로봇별 차이만 프로필에 보관합니다. 각 `runtime_manifest.json`은 원본 파일과 로봇 실행 위치, SHA-256을 기록합니다. 공유 파일 수정 시 `python3 scripts/update_manifest.py`로 해시를 갱신하고 검사를 실행하세요.

## 이름과 호환성

외부 로봇 ID와 Action 주소는 `M1` → `/m1/data`, `M2` → `/m2/data`입니다. 기존 검증된 내부 ROS namespace/TF·서비스는 첫 이관에서 `burger1`, `burger2`로 유지합니다. `TURTLEBOT3_MODEL=burger`도 하드웨어 모델이므로 바꾸지 않습니다. 이 연결은 각 `config/robot.yaml`에 기록합니다.

실행 디렉터리에는 호환성 때문에 `final_robot_ws/host_ws`, `handoff`라는 경로가 생성됩니다. **여기에 들어가는 것은 로봇에서 실행하는 ROS 패키지와 환경 설정이며, 호스트 PC 프로그램이 아닙니다.** 호스트 CLI·웹·RViz 실행 도구는 이 저장소에 포함하지 않습니다.

## 로봇에서 처음 구성

Ubuntu 24.04 / ROS 2 Jazzy와 해당 로봇의 `~/turtlebot3_ws/install/setup.bash`가 필요합니다. M1은 기존 `~/camera-build/libcamera/build/src/apps/cam/cam` 런타임을 사용합니다. M2 카메라 설치는 생성된 카메라 디렉터리의 `setup_camera.sh`를 사용하며 외부 소스 버전은 `dependencies.repos`에 고정했습니다. 기존 카메라 보정과 촬영 해상도를 유지해야 합니다.

```bash
cd ~/AMR
python3 scripts/materialize.py M2 --output "$HOME/amr_runtime/M2"
bash scripts/build.sh "$HOME/amr_runtime/M2"
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" install-services
```

생성기는 기존 출력 디렉터리를 덮어쓰지 않습니다. 기존 운용 서비스와 내용이 다르면 서비스 등록도 중단합니다. 이전 로봇 운용을 중지하고 유닛을 백업·검토한 뒤 전환하세요. 위 명령은 본 저장소를 로봇에 배치한 후 사용하는 절차이며, 이번 이관 작업에서 실제 로봇의 서비스를 교체하지 않았습니다.

M1은 위 명령의 `M2`만 `M1`으로 바꿉니다. 수정된 소스는 새 실행 디렉터리로 다시 구성·빌드해야 반영됩니다.

## 실행

[실행 명령](docs/commands.md), [호스트 Action 계약](docs/host_interface.md), [검증·제한](docs/validation.md), [남은 문제](docs/known_issues.md)를 확인하세요.

- 개별 시험: 로봇 내부 `operation.py`를 통해 경로를 하나씩 실행합니다.
- 호스트 연동: `process` 모드에서 Action Server를 실행합니다.
- `EMER_STOP`: 정지 래치를 설정하고 진행 중 작업을 중단합니다.
- `RESTART`: 신선한 odom으로 정지를 확인한 후 래치만 해제합니다. 이전 작업은 재개하지 않습니다.
- 성공 결과: 웨이포인트 접수가 아닌, 도킹/대기장소 IR 정지까지 검증된 `arrived`만 성공입니다.

새 Action 연동과 속도 제한은 격리된 ROS/모의 실행기로 검증했으며 실제 로봇에서의 통합 주행 검증은 남아 있습니다. 기존 본체 `stack smashing detected` / 종료 코드 `-6` 문제도 해결된 것으로 표시하지 않습니다.

## 테스트

```bash
python3 -m pytest -q tests/test_action_contract.py tests/test_backend.py tests/test_packaging.py
# host_pkg와 amr_mission 빌드 환경을 source한 뒤:
ROS_DOMAIN_ID=232 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 -m pytest -q tests/test_action_ros.py
```

지도·카메라 보정·마커 설정은 버전 관리합니다. 빌드 결과, 실행 로그, 원본 영상, 임시 출력, 과거 `.before` 백업, 외부 카메라 소스 전체는 올리지 않습니다.
