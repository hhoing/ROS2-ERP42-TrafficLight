# State Machine

이 문서는 `erp42/erp42_control/erp42_control/state_machine.py` 중심으로 전체 주행 상태 전환과 mission controller 연결 구조를 정리합니다.

## Role

`state_machine.py`는 전체 코스를 state 단위로 나누고, 현재 state에 맞는 제어 로직을 선택합니다. 일반 주행 구간에서는 Stanley path tracking을 수행하고, 미션 구간에서는 전용 controller에 제어권을 넘깁니다.

## State Definition

`State` enum은 코스 구간을 순서대로 정의합니다.

예시:

```text
A1A2   -> driving_a
A2A3   -> pickup_b
A5A6   -> obstacle_e
A10A11 -> traffic_light_i
A20A21 -> traffic_light_s
A33A34 -> parking_E
```

state 이름의 prefix는 구간 ID이고, value는 mission type과 suffix를 포함합니다. 코드에서는 `self.state.value[:-2]`를 사용해 `driving`, `curve`, `traffic_light` 같은 mission type을 판별합니다.

## Path Loading

`GetPath`는 `DB.query_from_id()`를 통해 현재 state ID에 해당하는 path를 읽습니다.

관리하는 path 값:

- `cx`: x 좌표 list
- `cy`: y 좌표 list
- `cyaw`: yaw list
- `cv`: target velocity list

state가 바뀌면 `path.file_open_with_id(self.state.name)`로 다음 구간 path를 다시 읽고, `global_path` 토픽으로 publish합니다.

## Odometry And Feedback

`GetOdometry`는 두 입력을 구독합니다.

- localization odometry topic
- `erp42_feedback`

odometry에서는 차량의 `x`, `y`, `yaw`를 가져오고, ERP42 feedback에서는 현재 속도 `v`를 읽습니다.

## Driving And Curve Control

일반 주행과 곡선 구간은 같은 기본 구조를 가집니다.

```text
odometry + path
  -> Stanley steering
  -> target speed from path
  -> heading/cross-track error based speed adaptation
  -> PI speed control
  -> ControlMessage
```

차이점:

- `driving`: speed limit이 더 높게 설정됨
- `curve`: max speed를 낮춰 곡선 구간 안정성을 높임

## Mission Controller Dispatch

`update_cmd_msg()`는 현재 state type에 따라 controller를 선택합니다.

| State type | Controller |
| --- | --- |
| `parking` | `self.parking.control_parking()` |
| `obstacle` | `self.obstacle.control_obstacle()` |
| `pickup` | `self.pickup.control_pickup()` |
| `delivery` | `self.delivery.control_delivery()` |
| `traffic_light` | `self.traffic_light.control_traffic_light()` |
| `stop_line` | `self.stop_line.control_stop_line()` |

각 mission controller는 `ControlMessage`와 `mission_finish` 값을 반환합니다. `mission_finish`가 `True`가 되면 state machine은 다음 state로 넘어가고 path를 갱신합니다.

## Traffic Light State Connection

신호등 state에서는 다음 호출이 핵심입니다.

```python
msg, self.mission_finish = self.traffic_light.control_traffic_light(self.odometry, self.path)
```

`Trafficlight` controller 내부에서 YOLOv4 기반 인식 결과를 안정화하고, 정지선까지 남은 거리와 함께 `speed`, `steer`, `estop`을 결정합니다. state machine은 이 결과를 그대로 `cmd_msg`로 publish합니다.

## Main Loop

`main()`은 ROS2 node를 만들고 다음 순서로 동작합니다.

```text
load parameters
  -> open path DB
  -> initialize odometry
  -> initialize StateMachine
  -> publish initial path
  -> spin node in background thread
  -> 10 Hz publish_cmd loop
```

## Known Issues

- `main()`의 초기 state가 코드상 `State.A32A33`으로 설정되어 있습니다. 전체 코스 시작인지 특정 테스트/대회 상황의 시작점인지 별도 설정화가 필요합니다.
- DB 파일명이 `BS_final.db`로 코드에 직접 선언되어 있습니다. 공개 저장소에서는 launch parameter 또는 config 문서로 분리하는 것이 좋습니다.
- `self.state.value[:-2]` 방식은 suffix 길이에 의존합니다. state naming convention이 바뀌면 dispatch가 깨질 수 있습니다.
- 일부 mission controller와 parameter 값은 특정 대회 환경에 맞춰 하드코딩되어 있습니다.
- 예외 처리에서 `print(ex)`만 수행하므로 실제 차량 운용 시 에러 상태를 명확히 publish하거나 fail-safe로 전환하는 로직이 필요합니다.
- backup/copy 파일과 날짜별 실험 파일은 공개 저장소에서 제외하는 것이 좋습니다.
