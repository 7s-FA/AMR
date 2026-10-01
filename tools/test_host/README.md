# 간단한 주행 테스트 호스트

ROS 2 Jazzy가 설치된 **호스트 PC**에서 M1 또는 M2를 한 대씩 시험합니다. 본 호스트의 공정 스케줄러 없이 동일한 `host_pkg/action/Burger`를 전송합니다. 로봇 펌웨어나 속도 토픽을 직접 제어하지 않습니다.

## 1. PC에서 최초 빌드

```bash
cd ~/AMR
bash tools/test_host/build.sh
```

인터페이스만 `~/AMR/.test_host_ws`에 빌드합니다. 기존 호스트 워크스페이스는 변경하지 않습니다.

## 2. 로봇에서 준비

먼저 [로봇 설치 안내](../../docs/commands.md)에 따라 최신 런타임을 생성·빌드하고 `amr` 함수를 설정합니다. 아래는 **로봇 터미널** 명령입니다.

```bash
amr mode process
# 실제로 지정 주차 위치와 방향이 맞는 경우에만: amr parked
amr ready
amr action
```

개별 시험용 호스트도 ROS Action으로 접수하므로 로봇은 `process` 모드를 사용합니다. `individual`은 로봇 내부 CLI용입니다. 여러 로봇이나 자동 공정을 실행할 필요는 없습니다. 이미 Action 서비스를 실행 중이라면 `amr action`을 중복 실행하지 않습니다.

## 3. PC에서 한 경로 전송

```bash
cd ~/AMR
export ROS_DOMAIN_ID=40
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER ROS_STATIC_PEERS
source tools/test_host/env.bash
M2 asm
```

`.bashrc` 수정은 필요 없습니다. 새 터미널에서는 위 설정을 다시 실행합니다. 운영 네트워크에서 별도 DDS 설정을 사용하는 경우 그 설정을 양쪽에 동일하게 적용합니다. 도메인 **232는 모의 자동 테스트 전용**이며 실물 연결은 양쪽 모두 **40**입니다.

명령을 하나씩 실행하고 최종 결과를 확인한 뒤 다음 명령을 보냅니다.

| 명령 | 동작 |
|---|---|
| `M2 mat` | 재료창고 이동 → 일반 도킹 |
| `M2 asm` | 조립대 이동 → 일반 도킹 |
| `M2 rest` | 대기장소 이동 → IR 감지·정지 확인 |
| `M2 park` | 웨이포인트 4 → 주차 도킹 |
| `M2 asm --speed 50` | 최대속도 상한 50%로 요청 |
| `M2 stop` | 비상정지 요청·정지 래치 |
| `M2 restart` | 정지 래치 해제만, 이전 이동 재개 없음 |
| `M2 status` | 이후 발생하는 진단 이벤트 구독, Ctrl+C로 종료 |
| `M1 asm` | M1 조립대 시험; 나머지 명령도 동일 |

`status`는 저장된 현재 상태 조회가 아니라 실시간 이벤트 구독입니다. 이동 명령 창에는 접수 ID, 위치 피드백, 진단 이벤트, 최종 성공/실패가 표시됩니다. Ctrl+C나 결과 대기 시간 초과 시 취소를 요청하며, 로봇은 취소 후 정지 래치를 유지합니다. 응답이 없으면 정지가 확인된 것이 아니므로 다른 터미널에서 `M2 stop`으로 확인합니다. 실패·취소 후 원인을 확인하고 `restart` 후 새 경로를 보냅니다.

함수 없이도 사용할 수 있습니다.

```bash
bash ~/AMR/tools/test_host/run.sh M2 GO_TO_ASM --speed 100
```

주소는 대소문자를 구분합니다: **`/M1/data`, `/M2/data`**. 센서/Nav2 내부 토픽은 `/burger1/...`, `/burger2/...`로 유지됩니다. 테스트 호스트와 본 공정 호스트를 동시에 명령 발행용으로 사용하지 않습니다.
