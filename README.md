# AMR · 자재와 팔레트 운반 로봇

M1·M2는 자재를 제작공정으로 운반하고, F1은 빈 팔레트 공급과 완제품 보관을 담당합니다. 상위 호스트가 목적지를 지정하면 각 로봇이 주행·도킹을 수행하고 결과를 돌려줍니다.

## 로봇별 역할

| 장치 | 하는 일 | 현재 장치 소스 |
|---|---|---|
| M1 | 자재창고 → 제작공정 운반 | [deployments/M1/source](deployments/M1/source) |
| M2 | M1과 교대로 자재 운반 | [deployments/M2/source](deployments/M2/source) |
| F1 | A/B 창고의 팔레트 공급 → 완제품 회수·보관 | [deployments/F1/source](deployments/F1/source) |

## 코드 보기

- `deployments/<장치>/source`: 각 장치에서 직접 수집한 실행 소스입니다. **장치 복원은 해당 폴더를 기준으로** 합니다.
- `deployments/<장치>/snapshot.json`: 수집 출처와 파일 해시입니다.
- 루트 `common`, `scripts`: 관제 PC의 공통 개발 사본입니다. M1·M2 장치별 공통 파일에는 차이가 있습니다.
- `F1/src/waffle`: F1 ROS 패키지 사본입니다.

## 연결 구조

M1·M2는 ROS 2 Action(`/M1/data`, `/M2/data`)으로 이동 명령과 결과를 주고받습니다. 현재 호스트의 F1 제어는 `/F1/manual/command`·`/F1/manual/status` JSON 토픽을 사용합니다.

[전체 통신 표](https://github.com/7s-FA/host_web/blob/host_total/README.md#통신-프로토콜) · [M1·M2 설치 구조](docs/single_folder.md) · [수집 범위와 별도 준비 파일](HOST_TOTAL.md)

이 브랜치는 `host_total` 통합 소스입니다. ROS 의존성·장치별 보정값·실행 환경은 별도 준비하며, 다른 로봇의 운행 상태나 빌드 결과를 복사하지 않습니다.
