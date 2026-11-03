# host_total · 2026-11-03

M1·M2는 각 장치의 `/home/ubuntu/AMR`에서, F1은 `/home/ubuntu/turtlebot3_ws/src/waffle`에서 직접 수집했습니다.

- M1 기준 실행 소스: [deployments/M1/source](deployments/M1/source)
- M2 기준 실행 소스: [deployments/M2/source](deployments/M2/source)
- F1 ROS 패키지: [deployments/F1/source](deployments/F1/source) (`F1/src/waffle`에도 제공)

장치별 `common`이 다르므로 M1 실행본을 M2에 덮어쓰지 마세요. 각 snapshot.json에 수집 파일의 SHA-256을 기록했습니다. 루트의 공통 소스는 관제 PC 작업 사본이며, 장치 복원 기준은 해당 deployments 소스입니다. 런타임·ROS 의존 패키지는 별도 설치해야 합니다.

이번 작업은 소스 게시만 수행했습니다. 장치 프로세스 재시작·이동·펌웨어 쓰기는 수행하지 않았습니다. 기존 브랜치 이력을 유지합니다.
