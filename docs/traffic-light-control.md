# Traffic Light Control

이 문서는 `erp42/erp42_control/erp42_control/controller_traffic_light.py` 중심으로 신호등 미션 제어 구조를 정리합니다.

## Purpose

신호등 미션의 목적은 YOLOv4 기반 신호등 인식 결과를 ROS2 제어 로직과 연동해 ERP42 차량 명령으로 변환하는 것입니다. 인식 결과를 바로 사용하지 않고, 현재 주행 구간에 맞는 class ID만 필터링하고 temporal voting으로 신호 상태를 안정화합니다.

## Input And Output

입력:

- `darknet_ros_msgs/BoundingBoxes`
- topic: `bounding_boxes`
- 각 bounding box의 `class_id`, `probability`

출력:

- `erp42_msgs/ControlMessage`
- topic: `cmd_msg`
- 주요 필드: `speed`, `steer`, `estop`, `gear`

## Section-Specific Class ID Filtering

`Trafficlight.traffic_light_sections`는 7개 신호등 구간에 대해 red/green class ID set을 따로 둡니다.

| Index | Red class IDs | Green class IDs |
| --- | --- | --- |
| 0 | `1401`, `1402` | `1405` |
| 1 | `1401`, `1402` | `1405` |
| 2 | `1401` | `1400`, `1405` |
| 3 | `1400`, `1401`, `1402`, `1404` | `1403` |
| 4 | `1301` | `1303` |
| 5 | `1401`, `1402` | `1405` |
| 6 | `1401`, `1402` | `1405` |

`get_current_traffic_light_ids()`는 `current_index`를 기준으로 현재 구간의 red/green ID set을 반환합니다. 이렇게 구간별 필터를 둔 이유는 코스 위치에 따라 필요한 신호등 방향과 class 의미가 달라질 수 있기 때문입니다.

## Temporal Voting

`traffic_light_callback()`은 YOLOv4 기반 인식 결과를 받아 다음 값을 누적합니다.

- red ID에 해당하면 `recent_signals.append("red")`
- green ID에 해당하면 `recent_signals.append("green")`
- 현재 구간과 무관한 ID면 `recent_signals.append("none")`

`get_real_light()`는 최근 관측치를 기반으로 `current_signal`을 확정합니다.

주요 조건:

- red만 감지되고 green이 없으면 red 확정
- green만 감지되고 red가 없으면 green 확정
- 최근 10개 관측 중 red 또는 green이 6회 이상이면 기존 신호에서 전환
- 최근 10개 관측이 모두 red 또는 green이면 해당 신호 확정
- red/green 양쪽 타임스탬프가 있을 때, 한쪽이 일정 시간 이상 더 최근이면 해당 신호 확정
- 이미 신호가 확정된 뒤 3초 이상 none 상태가 지속되면 신호 상태 초기화

이 방식은 단일 프레임 오검출이나 순간적인 미검출이 곧바로 차량 명령으로 이어지는 것을 줄이기 위한 안정화 로직입니다.

## Vehicle Command Mapping

`control_traffic_light()`는 path tracking과 신호 상태를 함께 사용합니다.

공통 조향:

- Stanley controller로 `steer` 계산
- `msg.steer = int(degrees(-steer))`

red:

- 정지선까지 남은 index가 10 이하이면 `speed = 0`, `estop = 1`
- 정지선 전이면 목표 속도를 낮추고 PI controller로 감속

green:

- 목표 속도 8 km/h 수준으로 설정
- heading/cross-track error 기반 속도 보정
- PI controller로 실제 속도 명령 계산
- 정지선 근처에서 `mission_finish = True`

unknown:

- 신호가 확정되지 않으면 제한 속도로 진행
- 정지선 근처에서 불가피한 미션 완료 처리

미션 완료 시:

- `current_index += 1`
- 신호 상태 초기화
- `state_machine.py`로 `mission_finish = True` 반환

## Design Point

핵심 설계 포인트는 perception 결과와 control command 사이에 mission context를 넣은 것입니다.

```text
raw detection
  -> section context
  -> class ID filter
  -> temporal voting
  -> traffic signal state
  -> vehicle command
```

이 구조 덕분에 특정 구간에서 필요한 신호등 class만 반영할 수 있고, 일시적인 detection fluctuation이 ERP42 `estop`이나 `speed` 명령으로 바로 튀는 것을 줄일 수 있습니다.

## Known Issues

- `current_index`의 초기값이 코드에서 `6`으로 설정되어 있어, 전체 코스 시작 기준인지 특정 테스트 시작 기준인지 문서/설정 분리가 필요합니다.
- 미션 완료 후 `current_index += 1`이 호출되므로 7개 구간 이후 index overflow 가능성을 방어하는 로직이 필요합니다.
- red 감속 branch에서 `self.speed`를 참조하는 코드가 보이지만 `Trafficlight` 클래스 내부에 해당 속성이 명확히 초기화되어 있지 않습니다. 실제 수정은 하지 않았고, 추후 코드 점검 대상으로 남깁니다.
- `confidence = box.probability`를 읽지만 현재 로직에서는 confidence threshold를 사용하지 않습니다.
- class ID mapping, vote threshold, time threshold가 코드에 하드코딩되어 있습니다. 공개/재사용 목적이라면 YAML 또는 ROS2 parameter로 분리하는 것이 좋습니다.
- `darknet_ros_yolov4` 전체 구조와 weights 파일은 이 저장소 정리 범위에서 제외합니다.
