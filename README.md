AMR/
├── M1/
│   ├── image/
│   ├── src/
│   └── map/
├── M2/
│   ├── image/
│   ├── src/
│   └── map/
├── M3/
│   ├── image/
│   ├── src/
│   └── map/
└── F1/
    ├── image/
    ├── src/
    └── map/
```

## Directory Description

- `M1/`, `M2/`, `M3/`: AMR 이동 로봇별 디렉터리
- `F1/`: Forklift Robot 관련 디렉터리

각 로봇 디렉터리는 다음과 같이 구성합니다.

- `image/`: 카메라 이미지 및 이미지 처리 관련 데이터 저장
- `src/`: ROS 2 노드 및 로봇 제어 소스 코드 저장
- `map/`: Nav2 및 SLAM에서 사용하는 지도 파일 저장
