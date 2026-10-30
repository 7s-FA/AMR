# F1 — TurtleBot3 Waffle

2026-10-30 와플의 소스에서 가져온 ROS 2 패키지입니다. `waffle/`에 실행 코드·설정·지도·카메라 보정값을 함께 보관하고, `robot3_interfaces/`에 `/F1/data`의 `Waffle` Action 정의를 보관합니다.

```text
F1/
├── waffle/
│   ├── waffle/              # Python 노드와 도킹 코드
│   ├── launch/              # system.launch.py
│   ├── config/              # Nav2·도킹·라이다 필터 설정
│   ├── calibration/         # .npy 2개와 .npz 1개
│   ├── factory_map.yaml
│   ├── factory_map.pgm
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── robot3_interfaces/
    ├── action/Waffle.action
    ├── CMakeLists.txt
    └── package.xml
```

`jeongje` 브랜치의 `02c0d23` 이력을 병합하고, 실제 와플에서 받은 최신 소스를 위 구조로 반영했습니다. 정제 브랜치에서 삭제한 빈 `image/`, `map/`, `src/` 디렉터리 표시는 되살리지 않습니다. 지도는 `waffle/`에, 최신 보정값은 `waffle/calibration/`에만 보관합니다.

## 원본과 반영 범위

- `ubuntu@waffle.local:/home/ubuntu/turtlebot3_ws/src/waffle/`
- `ubuntu@waffle.local:/home/ubuntu/turtlebot3_ws/robot3_interfaces/`
- TurtleBot3의 `turtlebot3_bringup/param/waffle.yaml`은 `waffle/config/turtlebot3_waffle.yaml`로 보관합니다. 원본과 설치본의 일치를 확인했으며, 현재 로봇의 `enable_stamped_cmd_vel: false` 설정을 그대로 사용합니다.
- `camera-test/`, 별도 `waffle_control/`, 빌드·설치 결과, 로그·캐시는 포함하지 않습니다. 실행 코드에서 참조하지 않는 `image.png`도 제외했습니다.
- 원본 대비 `waffle/setup.py`의 지도·보정 파일 설치 목록과 `waffle/launch/system.launch.py`의 리소스 경로를 수정했습니다. launch에서 보관한 TurtleBot3 설정을 `tb3_param_dir`로 전달합니다. 주행·도킹·리니어 제어 코드와 설정값은 원본을 유지합니다.

## 빌드

와플에서 확인한 ROS 배포판은 Jazzy입니다. 저장소 루트에서 다음과 같이 F1 패키지만 빌드합니다. 빌드는 로봇 노드를 실행하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
cd F1
colcon build --base-paths waffle robot3_interfaces --symlink-install
source install/setup.bash
```

로봇 실행에는 별도로 설치된 TurtleBot3 bringup·센서 드라이버, Nav2, `laser_filters`, OpenCV ArUco, NumPy, PyYAML, `gpiozero`·`lgpio`, `rpicam-vid`가 필요합니다. 해당 시스템 패키지는 이 복사본에 포함하지 않습니다. 현재 검출 코드는 OpenCV의 `aruco.DetectorParameters_create()`·`aruco.detectMarkers()` API를 사용합니다.

## 실행

와플에서 기존 실행 프로세스와 제어권 중복이 없는 상태로 사용합니다. 아래 환경값은 원본 와플의 설정입니다. 실행 시 리니어 초기 동작을 포함한 하드웨어 제어가 시작됩니다.

```bash
export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-03
export ROS_DOMAIN_ID=40
ros2 launch waffle system.launch.py
```

launch는 설치된 `waffle` 패키지에서 지도·설정을 찾고, 도킹 코드는 같은 패키지의 `calibration/`을 읽습니다. `/home/ubuntu/turtlebot3_ws/src/waffle` 경로에 의존하지 않습니다. 카메라는 도킹 코드가 `rpicam-vid`로 직접 열기 때문에 `camera-test/`는 필요하지 않습니다.

저장소의 `scripts/robot.py`·`scripts/materialize.py`는 M1·M2용입니다. F1은 위 명령으로 별도 빌드·실행하며 이번 추가에 배포 스크립트 통합은 포함하지 않습니다. PC 빌드 검증은 실제 와플의 주행·도킹·GPIO 동작 검증을 의미하지 않습니다.
