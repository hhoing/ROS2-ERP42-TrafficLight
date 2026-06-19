# Architecture

이 문서는 ROS2 ERP42 자율주행 프로젝트의 전체 구조를 포트폴리오 관점에서 요약합니다. 실제 코드 수정 없이 현재 워크스페이스의 패키지 구성과 주요 데이터 흐름을 설명합니다.

## System Overview

```text
Sensors
  -> localization / perception / clustering
  -> path and mission state machine
  -> mission-specific controller
  -> erp42_msgs/ControlMessage
  -> ERP42 serial communication
```

프로젝트는 ROS2 패키지 단위로 센서 처리, 위치 추정, 경로 추종, 미션 제어를 나눕니다. `erp42_control` 패키지가 주행 state machine과 mission controller를 묶고, 최종적으로 ERP42 차량 제어 메시지를 publish합니다.

## Main Packages

| Path | Role |
| --- | --- |
| `erp42/erp42_control` | State machine, Stanley tracking, mission controllers |
| `erp42/erp42_msgs` | ERP42 control/feedback custom messages |
| `sensor/erp42_communication` | ERP42 serial communication package |
| `localization` | Localization, map/odom TF, path-related utilities |
| `adaptive_clustering`, `adaptive_clustering_msgs` | LiDAR clustering utilities and messages |
| `obstacle_avoidance` | Obstacle mission control logic |
| `path_plan_cone` | Cone path planning utilities |
| `tf`, `tf_cone` | Coordinate transform utilities |
| `opencv_lane` | Camera lane processing experiments |

`darknet_ros_yolov4`는 YOLOv4 기반 인식 결과를 제공하는 외부 perception stack으로 사용된 것으로 정리합니다. 포트폴리오 저장소에는 전체 외부 구조와 weights 파일을 포함하지 않는 것을 전제로 합니다.

## Control Message

`erp42_msgs/msg/ControlMessage.msg`:

```text
uint8 mora
uint8 estop
uint8 gear
uint16 speed
int16 steer
uint8 brake
uint8 alive
```

`erp42_control`의 각 controller는 주행 상황에 맞춰 `ControlMessage`를 만들고 `cmd_msg` 토픽으로 publish합니다.

## State And Mission Flow

`erp42_control/state_machine.py`의 `State` enum은 전체 코스를 구간 단위로 나눕니다.

```text
driving / curve
  -> path tracking
  -> target index reaches end of section
  -> next state

mission states
  -> mission-specific controller
  -> mission_finish == True
  -> next state and path reload
```

일반 주행과 곡선 구간에서는 Stanley controller로 조향을 계산하고, PI controller와 error 기반 속도 보정으로 `speed`를 계산합니다. 미션 구간에서는 장애물, 픽업, 딜리버리, 주차, 정지선, 신호등 등 개별 controller가 명령을 생성합니다.

## Traffic Light Integration

신호등 미션은 `controller_traffic_light.py`에서 처리합니다.

```text
darknet_ros_msgs/BoundingBoxes
  -> current traffic-light section index
  -> red/green class ID set selection
  -> temporal voting
  -> current_signal
  -> ControlMessage
```

이 구조의 핵심은 YOLOv4 기반 인식 결과를 그대로 제어에 쓰지 않고, 구간별 class ID 필터링과 temporal voting을 거친 뒤 차량 명령으로 연결한다는 점입니다.

## Excluded From Portfolio Repository

다음 항목은 공개 저장소에 포함하지 않는 것을 권장합니다.

- YOLO weights, TensorRT engine, PyTorch checkpoint 등 대용량 모델 파일
- `darknet_ros_yolov4` 전체 외부 소스 트리
- `build/`, `install/`, `log/`
- 백업/복사본 파일
- zip 압축 파일
- 특정 PC 경로나 대회 당일 임시 설정 파일

## Known Issues

- 여러 패키지 안에 실험용 파일, 날짜 suffix 파일, backup/copy 파일이 섞여 있습니다.
- 외부 패키지와 직접 작성 패키지의 경계가 저장소 구조상 명확하지 않습니다.
- 실행 파라미터 일부가 launch/config 파일보다 Python 코드 내부에 직접 들어 있습니다.
- 공개 저장소에서는 third-party package를 submodule, 설치 안내, 또는 별도 dependency 문서로 분리하는 편이 유지보수에 유리합니다.
