# M1/M2 호스트 연동: 전원 투입 후 준비

2026-10-02 버거1에서 확인한 절차입니다. `/home/ubuntu/final_robot_ws`에 배포된 로봇 런타임 기준이며, 새 경로에 materialize한 설치는 해당 경로를 사용합니다. M2에도 같은 공통 Action 서버를 배포·빌드했습니다. 이는 M2 실주행 검증을 의미하지 않습니다. [현장 코드 동기화·검증 기록](robot_sync_2026-10-02.md)을 참고하세요.

호스트 공정 시작은 아래 준비가 끝난 뒤 수행합니다. 로봇은 자기 주차 시작 위치와 방향에 정지해 있어야 합니다. 중간 위치에서는 `parked`로 주차 좌표를 덮어쓰지 않습니다. 각 단계가 성공한 경우에만 다음 단계로 진행합니다.

## 1. 해당 로봇 터미널에서 환경 선택

M1은 `ROBOT_ID=M1`, M2는 `ROBOT_ID=M2`로 설정합니다.

```bash
ROBOT_ID=M1
case "$ROBOT_ID" in
  M1) RUNTIME_ROBOT=burger1; ACTION_UNIT=m1-action.service ;;
  M2) RUNTIME_ROBOT=burger2; ACTION_UNIT=m2-action.service ;;
esac
cd "/home/ubuntu/final_robot_ws/robot/$RUNTIME_ROBOT/navigation"
source ./nav_env.bash
```

## 2. 초기 위치와 주행 준비

```bash
python3 operation.py mode individual
python3 operation.py parked
python3 operation.py ready
```

`parked`는 초기 주차 자세와 다음 주행의 출차 표시를 설정합니다. `ready`는 본체·위치 추정·카메라·Nav2를 준비하며 이동 목표를 보내지 않습니다. `ready: true`를 확인합니다. 실패하면 원인을 확인하고 이후 단계를 진행하지 않습니다.

## 3. 모터 토크 확인

```bash
ros2 topic echo "/$RUNTIME_ROBOT/sensor_state" --once --field torque
```

`False`이고 로봇이 정지해 있으며 실행 중인 이동 작업이 없을 때만:

```bash
ros2 service call "/$RUNTIME_ROBOT/motor_power" std_srvs/srv/SetBool "{data: true}"
ros2 topic echo "/$RUNTIME_ROBOT/sensor_state" --once --field torque
```

서비스 성공과 센서의 `True`를 확인합니다. 토크 활성화와 Action 정지 잠금 해제는 별개입니다.

## 4. Action 서버 실행과 정지 잠금 해제

```bash
systemctl --user start "$ACTION_UNIT"
journalctl --user -u "$ACTION_UNIT" -n 10 --no-pager
```

이번 실행의 `Robot action ready` 로그를 확인하고 로봇을 정지 상태로 잠시 유지한 뒤 아래 명령을 실행합니다. 서버가 시작되자마자 보내면 아직 정지 데이터를 모으지 못해 실패할 수 있습니다. 서버는 0.3초 이내의 odom으로 0.3초 이상 연속 정지를 확인해야 합니다. 고정 대기만으로 odom 수신을 보장할 수는 없습니다.

```bash
ros2 action send_goal "/$ROBOT_ID/data" host_pkg/action/Burger \
  "{command: RESTART, cmd_val: 0.0}"
```

`success: true`와 최종 성공 상태를 확인한 뒤:

```bash
python3 operation.py mode process
journalctl --user -u "$ACTION_UNIT" -f -n 20
```

`RESTART`는 잠금만 해제하며 이전 이동을 재개하지 않습니다. `FRESH_STATIONARY_ODOMETRY_REQUIRED`가 시작 직후 발생했다면 정지 상태에서 잠시 후 재시도합니다. 반복되면 odom 수신·시각·속도를 점검해야 하며 JSON 파일을 직접 수정해 우회하지 않습니다. `Ctrl+C`는 로그 보기만 종료합니다. Action 서버를 별도 터미널에서 중복 실행하지 않습니다.

호스트는 ROS_DOMAIN_ID=40과 `/M1/data` 또는 `/M2/data`, `host_pkg/action/Burger`를 사용합니다. 다른 호스트/시험 클라이언트에서 동시에 이동 명령을 보내지 않습니다.

## 확인된 변경과 남은 문제

- PC 공통 `action_server.py`의 `feedback()`에 사용자가 추가한 logger/print 출력을 유지합니다. 실제 수신 콜백의 로그가 아니므로 `pose: None`도 출력할 수 있습니다. `print()`는 버퍼링될 수 있습니다.
- M1에서 해당 파일을 배포·빌드했고 PC 소스와 로봇 로딩 파일의 SHA-256 일치를 확인했습니다. 그 시점 관련 격리 테스트는 48개 통과했습니다. 실주행 성공을 의미하지 않습니다.
- Action 서버가 AMCL보다 늦게 시작하고 새 `amcl_pose`가 없으면 위치가 `None`으로 남을 수 있습니다. 현재 구독은 과거 메시지를 받지 않고, 1초보다 오래된 위치는 거부합니다. TF 기반 연속 Feedback 수정은 아직 구현하지 않았습니다.
- M1에서 OpenCR 통신 오류 뒤 본체 종료(-6), 저전압 이력, Nav2 제어 서버 heartbeat 단절 및 컨테이너 종료(-11)를 관측했습니다. 재시작으로 일시 복구했지만 근본 해결은 확인되지 않았습니다.
- 11:30 로그에서 서버 준비 약 0.08초 뒤 `RESTART`가 실패했습니다. 시작 직후 정지 검증 시간이 부족했을 가능성이 있으며, 당시 재시도 성공은 확인하지 못했습니다.
- 초기 위치·토크·서비스·정지 잠금은 로봇별 실행 상태입니다. M1의 data 디렉터리를 M2로 복사하지 않으며 각 로봇에서 위 절차를 수행합니다.
