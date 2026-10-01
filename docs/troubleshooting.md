# 트러블슈팅

| 증상 | 확인과 조치 |
|---|---|
| UNKNOWN_COMMAND / goal 거부 | GO_TO_* 대문자 명령, speed 범위, process 모드, 기존 작업·정지 래치 확인 |
| ESTOP_LATCHED | 원인을 확인하고 실제 정지·odom 정상 상태에서 RESTART. 이전 작업 자동 재개 없음 |
| ZERO_SPEED_CANNOT_COMPLETE_ROUTE | 이동 goal의 cmd_val은 0보다 커야 함. 정지는 EMER_STOP 사용 |
| Action 서버 없음 | host_pkg 동일 빌드 여부, ROS_DOMAIN_ID=40, /m1/data 또는 /m2/data, 로봇 네트워크 설정 확인 |
| INITIAL POSE / 초기 위치 없음 | 실제 지정 주차 자세에서만 parked. 다른 위치는 정확한 현재 위치를 별도로 지정 |
| ready 시간 초과 | nav-control와 nav2 journal 확인. 기본 1회 Nav2 복구 유지, 위치 추정 재설정으로 우회하지 않음 |
| VISION_WAIT | 영상 공백 동안 정지 중. 최대 2초 기다리고 새 관측·정지 확인 후 재개 |
| odometry_timeout / 정지 확인 실패 | 본체 통신·프로세스 상태·odom/TF 지연 확인. watchdog을 늘려 우회하지 않음 |
| Spin 703 | nav2_failures 자료 확인. 정지 후 자료만으로 실제 장애물 원인을 단정하지 않음 |
| 낮은 속도에서 실패 | 모터 최소속도·양자화·기존 작업 시간 제한 확인. 새 속도 범위는 실기 검증 필요 |
| materialize 해시 불일치 | 수정 파일을 검토하고 update_manifest.py 실행. 로봇 실행 디렉터리에서의 임의 수정 대신 원본 소스 변경 |
| service 이미 존재 | 기존 작업과 유닛을 확인·백업 후 전환. 두 모터 제어 스택을 동시에 실행하지 않음 |

상세 원인은 `/m1/mission/diagnostics` 또는 `/m2/mission/diagnostics`, action/mission/base/nav2 journal 및 실행 디렉터리의 작업 JSON에 남습니다. Action의 IDLE 피드백은 정상 상태 문자열이며 도착 완료 판정은 Result로 합니다.
