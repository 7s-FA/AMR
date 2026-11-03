# 엔코더 드라이버 기준 소스

> 2026-09-29: Burger2 원본이 아래 before_patch와 바이트 단위로 일치함을 확인하고, 백업 후 최종 두 파일을 적용해 turtlebot3_node를 재빌드했다. [배포 기록](../../../docs/BURGER2_DEPLOYMENT_20260929.md). 아래는 최초 보관 시점의 기록이다.

출처: `주행로봇_인계_20260928/robot/encoder_patch`의 Burger1 적용본.
이 프로젝트의 공통 Nav2 기준을 재현할 때 검토할 TurtleBot3 드라이버 변경이다.

- `turtlebot3_node/`: 인계 당시 최종 `joint_state.cpp`와 `joint_state.hpp`.
- `before_patch/`: 인계 자료의 2026-09-28 12:24:09 변경 전 백업.
- `encoder_recovery_test.cpp`: 인계 자료의 필터 검사 코드.
- `joint_state.patch`: 위 변경 전 소스와 최종 소스의 차이.

두 로봇이 같은 드라이버 버전을 사용하도록 관리할 기준 자료다. **보관과 적용은 별개**다.
Nav2·도킹 배포 스크립트는 이 파일을 TurtleBot3 작업공간에 설치하거나 빌드하지 않는다.
Burger1은 인계 당시 적용됐다고 기록되어 있고, Burger2에는 이번 작업에서 적용하지 않았다.
현재 실행 바이너리는 원격으로 확인하지 않았다.

적용 전 각 로봇의 `~/turtlebot3_ws` 버전·원본 차이·장치를 비교해야 한다.
공통 Nav2 패키지 빌드만으로 이 드라이버는 바뀌지 않는다.
값 거부 시 odom 발행 중단, 카운터 초기화/긴 공백 이후 복구 제한,
통신 실패 시 캐시 값의 재발행 가능성은 별도 검증 항목이다.
검토 결과는 프로젝트 `output/encoder_review/results.txt`에 있다.
