# ROS2 ERP42 Autonomous Driving Control Project

ROS2 기반 ERP42 자율주행 프로젝트입니다. LiDAR, GNSS/IMU, camera perception, path tracking, mission state machine을 조합해 ERP42 차량의 `speed`, `steer`, `estop` 제어 명령으로 연결하는 구조를 갖습니다.

이 저장소 정리는 GitHub 포트폴리오 공개를 목표로 하며, 핵심 제어 로직과 신호등 미션 구조를 이해하기 쉽도록 문서화했습니다. 신호등 미션은 YOLOv4 기반 인식 결과를 ROS2 제어 로직과 연동한 구조입니다.

## Highlights

- ROS2 패키지 기반 ERP42 자율주행 제어 워크스페이스
- Stanley controller 기반 경로 추종
- PI speed control 및 heading/cross-track error 기반 속도 보정
- state machine 기반 일반 주행, 곡선, 장애물, 정지선, 주차, 픽업/딜리버리, 신호등 미션 전환
- YOLOv4 기반 신호등 인식 결과를 구간별 class ID 필터링과 temporal voting으로 안정화
- 안정화된 신호 상태를 ERP42 `speed`, `steer`, `estop` 명령으로 연결
- 대회 당일 7개 신호등 구간을 오작동 없이 통과

## Repository Scope

포트폴리오 공개용 저장소에는 핵심 ROS2 패키지와 문서만 포함하는 것을 전제로 합니다.

포함하지 않는 항목:

- `darknet_ros_yolov4` 전체 외부 구조
- YOLO weights 등 대용량 모델 파일
- `build/`, `install/`, `log/` 등 colcon 빌드 산출물
- 백업, 복사본, zip 압축 파일
- 하드코딩된 개인 로컬 경로가 포함된 임시 파일

위 항목은 `.gitignore`에 정리했습니다.

## Project Structure

```text
.
├── erp42/
│   ├── erp42_control/          # ERP42 mission/state/control logic
│   └── erp42_msgs/             # ControlMessage, SerialFeedBack messages
├── localization/               # Localization and TF utilities
├── sensor/                     # ERP42 communication, GNSS/IMU related packages
├── obstacle_avoidance/         # Obstacle mission logic
├── path_plan_cone/             # Cone path planning utilities
├── adaptive_clustering/        # LiDAR clustering
├── tf/, tf_cone/               # Coordinate transform utilities
├── docs/                       # Portfolio documentation
└── README.md
```

## Traffic Light Mission Flow

핵심 파일:

- `erp42/erp42_control/erp42_control/controller_traffic_light.py`
- `erp42/erp42_control/erp42_control/state_machine.py`
- `erp42/erp42_msgs/msg/ControlMessage.msg`

신호등 미션은 다음 흐름으로 동작합니다.

```text
YOLOv4 detection
  -> darknet_ros_msgs/BoundingBoxes
  -> section-specific red/green class ID filtering
  -> temporal voting over recent detections
  -> stable signal state: red / green / unknown
  -> Stanley steering + PI speed control
  -> ERP42 ControlMessage(speed, steer, estop, gear, brake)
```

`controller_traffic_light.py`는 현재 신호등 구간 인덱스에 따라 허용할 red/green class ID를 다르게 적용합니다. 같은 class ID라도 구간에 따라 의미가 달라질 수 있기 때문에, 전체 신호등을 하나의 공통 class mapping으로 처리하지 않고 구간별 필터를 둔 구조입니다.

이후 최근 인식 결과를 누적해 일시적인 오검출을 줄입니다. 예를 들어 최근 10회 중 6회 이상 같은 결과가 나오거나, 10회 연속 같은 결과가 나오면 신호 상태를 확정합니다. 확정된 신호는 정지선까지의 남은 거리와 함께 ERP42 제어 명령으로 변환됩니다.

## ERP42 Control Output

ERP42 제어 명령은 `erp42_msgs/msg/ControlMessage.msg`를 통해 publish됩니다.

주요 필드:

- `speed`: ERP42 속도 명령
- `steer`: 조향 명령
- `estop`: 신호등 red 정지 상황에서 비상 정지 플래그로 사용
- `gear`: 주행 기어
- `brake`: 감속/제동 명령

신호등 red가 확정되고 정지선에 가까워지면 `speed = 0`, `estop = 1`로 차량을 정지시킵니다. green이 확정되면 Stanley controller로 조향을 계산하고 PI controller로 속도를 조정해 주행을 이어갑니다.

## Build

ROS2 워크스페이스 루트에서 빌드합니다.

```bash
colcon build
source install/setup.bash
```

환경에 따라 외부 의존 패키지와 메시지 패키지가 필요합니다.

- `rclpy`
- `nav_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `tf_transformations`
- `darknet_ros_msgs`
- `erp42_msgs`

## Run

state machine 노드를 실행합니다.

```bash
ros2 run erp42_control state_machine
```

신호등 미션 단독 디버깅 시:

```bash
ros2 run erp42_control controller_traffic_light
```

실제 차량 주행 전에는 localization, ERP42 serial communication, perception 토픽, path DB가 모두 정상 publish되는지 확인해야 합니다.

## Documentation

- [Architecture](docs/architecture.md)
- [Traffic Light Control](docs/traffic-light-control.md)
- [State Machine](docs/state-machine.md)

## Known Issues

- 일부 파일명에 `backup`, `copy`, 날짜 suffix가 포함된 실험 사본이 있습니다. 공개 저장소에서는 제외하는 것을 권장합니다.
- 대용량 YOLO weights와 `darknet_ros_yolov4` 전체 외부 구조는 포트폴리오 저장소 범위에서 제외합니다.
- 일부 실행 설정은 특정 대회 환경의 DB 파일명, state 시작점, class ID mapping에 맞춰져 있습니다.
- 하드코딩된 mission index, class ID, path DB 설정은 향후 YAML/parameter 기반 설정으로 분리하는 것이 좋습니다.
- 코드 수정 없이 문서화만 진행했기 때문에, 잠재 버그는 `docs/traffic-light-control.md`와 `docs/state-machine.md`의 Known Issues에 따로 정리했습니다.
