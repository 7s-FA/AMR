# 로봇 내부 실행 명령

[전체 README](../README.md) · [호스트 Action 계약](host_interface.md)

**아래 설치·개별 시험 명령은 해당 로봇의 터미널에서 실행합니다.** 호스트 PC의 기존 `burger2` alias와는 별개입니다. M1과 M2 각각 설치하며, 한 번에 한 로봇의 경로만 시험해도 됩니다.

### 1. 저장소와 실행 대상 선택

Ubuntu 24.04 / ROS 2 Jazzy, `colcon`, Python YAML 모듈과 해당 로봇의 `~/turtlebot3_ws/install/setup.bash`가 필요합니다.

처음 내려받을 때만:

```bash
git clone --branch jh https://github.com/7s-FA/AMR.git "$HOME/AMR"
```

이후 해당 로봇의 터미널에서 아래 값을 선택합니다. **M1에서는 첫 줄만 `ROBOT_ID=M1`로 바꿉니다.**

```bash
ROBOT_ID=M2
AMR_RUNTIME="$HOME/amr_runtime/$ROBOT_ID"
AMR_ACTION="/${ROBOT_ID,,}/data"
if [[ "$ROBOT_ID" == M1 ]]; then
  RUNTIME_ROBOT=burger1
  AMR_CAMERA=final_robot_camera_burger1
else
  RUNTIME_ROBOT=burger2
  AMR_CAMERA=final_robot_camera
fi
cd "$HOME/AMR"
amr() { python3 "$HOME/AMR/scripts/robot.py" --runtime "$AMR_RUNTIME" "$@"; }
```

`amr`은 이 터미널에서만 쓰는 편의 함수입니다. 새 터미널을 열면 위 설정 블록을 다시 실행합니다. 이후 모든 명령은 선택한 로봇의 실행 디렉터리를 사용합니다.

### 2. 최초 구성·카메라 준비·빌드

```bash
python3 scripts/materialize.py "$ROBOT_ID" --output "$AMR_RUNTIME"
bash "$AMR_RUNTIME/$AMR_CAMERA/setup_camera.sh"
bash scripts/build.sh "$AMR_RUNTIME"
amr install-services
```

- M1의 카메라 준비는 기존 `~/camera-build/libcamera/build/src/apps/cam/cam` 실행 파일을 확인합니다. 기존 보정에 사용한 런타임이 필요합니다.
- M2는 새 실행 디렉터리 안에 고정된 버전의 libcamera·camera_ros를 설치·빌드합니다. 카메라 설치 과정에서 sudo 비밀번호를 요청할 수 있습니다.
- 생성기는 기존 출력 디렉터리를 덮어쓰지 않습니다. 이미 구성했다면 재실행하지 않고 다음 단계로 진행합니다. 소스 갱신 후에는 새 출력 경로를 사용합니다.
- 서비스 등록은 다른 내용의 기존 서비스를 덮어쓰지 않습니다. 기존 운용을 중지하고 유닛을 백업·검토한 뒤 전환해야 합니다. 등록만으로 서비스가 자동 시작되거나 로봇이 움직이지 않습니다.

### 3. 개별 시험 준비

```bash
amr mode individual
```

**로봇이 지정 주차 위치·방향에 실제로 놓여 있을 때만** 초기 위치를 확인합니다. 중간 지점에 있는 로봇에 이 명령을 사용하면 안 됩니다.

```bash
amr parked
```

Nav2·카메라를 준비하고 상태를 확인합니다. `ready`는 주행 명령이 아닙니다.

```bash
amr ready
amr status
```

### 4. 경로를 하나씩 실행

아래 표에서 원하는 명령 **하나만** 실행하고 `amr status`로 완료를 확인한 뒤 다음 경로를 요청합니다.

| 명령 | 동작 |
|---|---|
| `amr mat` | 재료 창고 이동·도킹 |
| `amr asm` | 조립대 이동·도킹 |
| `amr rest` | 대기장소 이동·IR 정지 |
| `amr park` | 주차 위치 이동·주차 도킹 |
| `amr status` | 모드·진행 단계·성공/실패·busy 확인 |
| `amr stop` | 현재 작업 중단 요청 |

예시:

```bash
amr asm
amr status
```

로컬 경로 명령은 접수 결과를 반환합니다. `status`의 작업 결과가 성공이고 `busy`가 false인지 확인합니다. `parked`는 초기 위치 확인, `park`는 실제 주차 이동입니다. `amr stop`은 로컬 작업 중단이고, Action `EMER_STOP`은 정지 래치와 odom 정지 확인까지 수행합니다.

### 5. 호스트 공정 모드로 전환

진행 중 작업이 끝난 뒤 로봇 터미널에서:

```bash
amr mode process
amr ready
amr action
```

`amr action`은 터미널을 점유하는 Action 서버입니다. 서비스로 실행하려면 **마지막 줄 대신** 아래 명령을 사용합니다. 두 방식을 동시에 실행하지 않습니다.

```bash
systemctl --user start "${ROBOT_ID,,}-action.service"
```

개별 시험으로 돌아갈 때는 진행 중 작업을 먼저 종료하고, 서버를 실행한 터미널에서 Ctrl+C 또는 서비스 방식일 경우 `systemctl --user stop "${ROBOT_ID,,}-action.service"`로 종료한 다음 `amr mode individual`을 실행합니다. 정지 래치가 남아 있다면 서버를 종료하기 전에 원인을 점검하고 아래 `RESTART` 절차로 해제합니다.

### 6. Action 명령 직접 시험

로봇의 **다른 터미널**에서 1번의 대상 선택 블록을 다시 실행하고 ROS 환경을 불러옵니다. 호스트 PC에서는 호스트에 빌드한 동일한 `host_pkg` 환경을 사용하며, 아래 로봇 전용 경로를 그대로 사용하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source "$AMR_RUNTIME/final_robot_ws/host_ws/install/setup.bash"
export ROS_DOMAIN_ID=40
ros2 action list -t
```

실제 조립대 이동을 요청하는 예시:

```bash
ros2 action send_goal "$AMR_ACTION" host_pkg/action/Burger \
  '{command: GO_TO_ASM, cmd_val: 100.0}' --feedback
```

다른 목적지는 command를 `GO_TO_MAT`, `GO_TO_REST`, `GO_TO_PARK` 중 하나로 바꿉니다.

정지가 필요할 때:

```bash
ros2 action send_goal "$AMR_ACTION" host_pkg/action/Burger \
  '{command: EMER_STOP, cmd_val: 0.0}'
```

실패 원인을 점검하고 실제 정지를 확인한 뒤 해제할 때:

```bash
ros2 action send_goal "$AMR_ACTION" host_pkg/action/Burger \
  '{command: RESTART, cmd_val: 0.0}'
```

`RESTART`는 이전 작업을 재개하지 않습니다. odom이 없거나 정지 확인이 안 되면 해제가 실패합니다. 이 명령은 소프트웨어 정지 제어이며 물리적 비상정지 회로를 대체하지 않습니다.

### 7. 상태와 로그

```bash
amr status
journalctl --user -u "${ROBOT_ID,,}-action.service" \
  -u "$RUNTIME_ROBOT-mission.service" -n 100 --no-pager
journalctl --user -u "$RUNTIME_ROBOT-base.service" \
  -u "$RUNTIME_ROBOT-nav2.service" -n 100 --no-pager
ros2 topic echo "/${ROBOT_ID,,}/mission/diagnostics"
```

| 기록 | 생성 위치 |
|---|---|
| 경로 명령별 결과 | `$AMR_RUNTIME/final_robot_ws/data/$RUNTIME_ROBOT/commands/` |
| 종단 도킹·대기 로그 | `$AMR_RUNTIME/final_robot_ws/data/$RUNTIME_ROBOT/terminal_logs/` |
| Action 상세 원인 | `/${ROBOT_ID,,}/mission/diagnostics` 토픽과 Action 서비스 journal |
