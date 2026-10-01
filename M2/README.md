# M2 로봇 프로필

기존 burger2의 지도·좌표·초기 자세·카메라 보정·주차 마커를 보존합니다.

- `config/`: 로봇별 설정. 외부 ID는 M2, 내부 namespace는 burger2입니다.
- `map/`: 실제 주행 지도와 메타데이터
- `src/`: 카메라·진단 등 이 로봇에 필요한 구현 차이
- `overrides/`: 공통 ROS 패키지와 다른 검증된 파일
- `runtime_env/`: 기존 로봇 스크립트의 환경 파일 이름 호환
- `runtime_manifest.json`: 공통 파일과 프로필을 실행 디렉터리로 배치하는 명세

설치·실행은 [전체 README](../README.md)와 [명령어](../docs/commands.md)를 따릅니다. 프로필 원본 폴더에서 구형 스크립트를 직접 실행하지 말고 materialize.py가 만든 디렉터리를 사용하세요. `image/`는 마커 도안과 설치 참고 자료용이며 원본 영상 로그를 넣지 않습니다.
