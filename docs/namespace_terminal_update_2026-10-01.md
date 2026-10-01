# M1/M2 네임스페이스·완료 판정·테스트 호스트 정리

2026-10-01. 이전 동기화 이후 추가 변경입니다.

## 로직

`/M1/data` 또는 `/M2/data`의 Burger Action → 명령·속도·모드·busy 확인 → 작업 ID 발급/접수 → 저장 경로의 웨이포인트 주행 → routes.yaml의 terminal 선택 → dock/park/rest 실행 → 컨트롤러 보고와 종료 코드로 정지 검증 → 최종 Action Result.

일반 도킹은 M1/M2가 같은 컨트롤러와 control 설정을 사용합니다. M2 park는 웨이포인트 4에서 주차 도킹으로 진행합니다. 주차 마커·지도·경로·카메라 보정 등은 로봇별 설정을 유지합니다.

## 수정

- Action 노드 namespace와 외부 주소를 대문자 `/M1`, `/M2`로 통일. 센서/Nav2 내부의 burger1/burger2는 그대로 연결합니다.
- REST가 IR 감지 직후 성공을 반환하던 것을 수정. IR 감지 후 0.5초 연속 정지, 0.3초 이내의 최신 odom 필요. IR 해제·odom 끊김·정지 확인 3초 초과는 실패.
- REST 속도·제한을 `rest_control`로 분리. 기본 직진 속도 0.039m/s 유지.
- 종료 코드 0만으로 departure_pending/도착 성공을 기록하지 않음. 도킹은 DOCKED/ir_high_and_stationary, REST는 RESTED/ir_high_and_stationary와 정지·IR 증거 필요.
- 사용되지 않던 `routes.yaml: terminal`을 실제 실행 기준으로 연결. 잘못된 값은 웨이포인트 이동 전에 거부.
- 중복 parking_marker_ids 삭제. 실제 주차 보드 파일이 유일한 마커 정의.
- Action heartbeat 갱신을 타이머 한 곳에서 수행. 실행 루프는 소유권·정지 래치만 읽고, 모터의 heartbeat 만료 보호는 유지.
- 사용되지 않던 M1 camera_node는 tools/legacy_camera로 보관하고 배포 대상에서 제외. 불필요한 start_nav_base와 M2 출차 중복 래퍼 삭제.
- 별도 주행 시험용 `tools/test_host` 추가. 본 호스트 저장소 수정 없이 M1/M2 Action에 직접 명령·취소·정지·해제 가능.

수동 지도 작성·진단 도구는 필요한 운영 도구이므로 남겼습니다.

## 검증과 범위

자동 테스트 총 **41개 통과**, 최초 41개 + odom 전송 지연 확인 신규 1개·기존 1개 재실행(상한 50개). 설정/정지/결과 증거/쉘 래퍼/백엔드/패키징 29회, M1/M2 ROS Action 및 테스트 호스트 실제 클라이언트↔모의 서버 통신 14개입니다. ROS 검사는 232·LOCALHOST에서 실행했으며 모터 명령은 보내지 않았습니다. 테스트 호스트의 host_pkg 빌드도 통과했습니다.

ROS 테스트 종료 시 기존 rclpy Destroyable 경고가 일부 출력됩니다. 테스트 실패는 아니며, 실제 주행 검증 완료를 뜻하지도 않습니다. 실제 로봇에 새 런타임 배포 후 REST odom 정지 확인과 각 경로를 확인해야 합니다. 기존 본체 stack smashing / -6 문제 역시 이 변경으로 해결된 것으로 보지 않습니다.

PC의 final_robot_ws/robot/burger1, burger2에는 공통 종단 실행·REST·경로 선택 변경을 반영하고 이전 파일을 output/namespace_terminal_fix_*에 백업했습니다. 원본 수집 manifest는 원본 기록으로 유지합니다. 최상위 host_ws/src는 변경하지 않았습니다. 대문자 Action과 테스트 호스트의 배포 기준은 AMR 저장소입니다. 실제 로봇은 변경하지 않았습니다.

[실행 명령: 테스트 호스트](../tools/test_host/README.md) · [로봇 설치 및 실행](commands.md)
