# 로봇 내부 실행 명령

아래는 M2 로봇 터미널 기준입니다. M1은 실행 디렉터리와 로봇 ID를 M1으로 바꿉니다. 호스트 PC용 `burger2` alias는 포함하지 않습니다.

## 구성·빌드·서비스 등록

```bash
cd ~/AMR
python3 scripts/materialize.py M2 --output "$HOME/amr_runtime/M2"
bash scripts/build.sh "$HOME/amr_runtime/M2"
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" install-services
```

출력 경로가 이미 있으면 새 경로를 선택합니다. 기존 서비스가 다른 실행 경로를 사용 중이면 등록기가 덮어쓰지 않습니다. 기존 작업을 중지하고 서비스 파일을 백업·검토한 뒤 교체해야 합니다. 자동으로 모터를 움직이거나 구형 서비스를 종료하지 않습니다.

## 개별 시험

```bash
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" mode individual
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" ready
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" status
```

경로는 다음 중 하나씩 실행합니다. 로컬 명령은 접수 결과를 반환하므로 status로 실제 완료를 확인한 뒤 다음 경로를 보냅니다.

```bash
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" asm
# 다른 경로: mat / rest / park
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" status
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" stop
```

`parked`는 실제로 지정 주차 위치·방향에 놓여 있을 때만 적용하는 초기 위치 확인입니다. 주차 이동은 `park`입니다.

## 호스트 Action 연동

```bash
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" mode process
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" ready
python3 scripts/robot.py --runtime "$HOME/amr_runtime/M2" action
```

서비스로 띄우려면 마지막 명령 대신 `systemctl --user start m2-action.service`를 사용합니다. 두 방식을 동시에 실행하지 않습니다. 서비스 등록만으로 서버가 자동 실행되지는 않습니다.

다른 ROS 터미널에서 통신 시험 예시(해당 ROS 환경·도메인 40 source 필요):

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/amr_runtime/M2/final_robot_ws/host_ws/install/setup.bash"
export ROS_DOMAIN_ID=40
ros2 action list -t
# 실제 주행을 요청하므로 현장 준비가 된 경우에만 실행
ros2 action send_goal /m2/data host_pkg/action/Burger \
  '{command: GO_TO_ASM, cmd_val: 100.0}' --feedback
```

정지·해제는 별도 터미널에서 보냅니다.

```bash
ros2 action send_goal /m2/data host_pkg/action/Burger \
  '{command: EMER_STOP, cmd_val: 0.0}'
ros2 action send_goal /m2/data host_pkg/action/Burger \
  '{command: RESTART, cmd_val: 0.0}'
```

RESTART는 이전 작업을 다시 실행하지 않습니다. Action 취소·실패 후에도 래치를 확인하고 필요 시 해제해야 합니다. odom이 없거나 정지 확인이 되지 않으면 해제 요청은 실패합니다.

## 로그

```bash
journalctl --user -u m2-action.service -u burger2-mission.service -n 100 --no-pager
journalctl --user -u burger2-base.service -u burger2-nav2.service -n 100 --no-pager
ros2 topic echo /m2/mission/diagnostics
```

작업 기록은 생성된 `final_robot_ws/data/burger2/commands/`, 종단 로그는 `terminal_logs/`에 남습니다. 상태 확인만 종료하거나 호스트 연결이 끊긴 것과 실제 로봇 정지는 다릅니다.
