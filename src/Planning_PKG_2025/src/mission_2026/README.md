# 2026 판단 미션 핵심 알고리즘

이 폴더는 2026 대회용 두 미션의 **ROS 비종속 핵심 로직과 ROS1 연결부**를 담는다.

- GPS 음영 터널
- 정적 장애물 + 신호등

핵심 알고리즘은 ROS 메시지를 직접 사용하지 않고, `mission_2026_ros1_adapter`만 ROS1 토픽을 변환한다. 따라서 rosbag 재생, 단위 테스트, 합성 입력 테스트에서 같은 알고리즘을 그대로 실행할 수 있다.

## 파일 구성

### `mission_types.hpp`

두 미션이 공통으로 사용하는 자료형을 정의한다.

- `Point2D`, `Pose2D`: 센서와 위치 입력
- `PathPoint`, `RelativePath`: 터널 상대경로
- `FrenetPoint`: 정적 장애물용 경로 기준 좌표
- `TrafficSignal`: `UNKNOWN`, `RED`, `YELLOW`, `GREEN`
- `ObstacleObservation`, `RoadBounds`, `StopLine`

`RelativePath::toPlanningPositionArray()`는 기존 제어 코드가 상대경로로 인식하는 `[-82.82, -82.82]` 마커를 자동으로 앞에 추가한다.

### `mission_math.hpp/.cpp`

공통 수학 기능을 제공한다.

1. MAD 기반 이상치 제거를 포함한 벽 직선 적합
2. 각도 정규화 및 저역통과 필터
3. 5차 smooth-step 함수와 1·2차 미분
4. 차량 기준 상대 로컬패스 생성
5. 경로 yaw와 곡률 계산

벽 직선 적합은 무작위 RANSAC 대신 결정론적 robust fit을 사용한다. 같은 rosbag을 재생하면 항상 같은 결과가 나오므로 회귀시험에 유리하다.

### `gps_shadow_tunnel.hpp/.cpp`

`GpsShadowTunnel::update()`에 `TunnelInput`을 전달하면 `TunnelOutput`이 반환된다.

상태:

```text
IDLE
→ CALIBRATING
→ WALL_TRACKING
→ GPS_RECOVERY
→ DONE
```

벽 추적 모드:

```text
DUAL_WALL
LEFT_WALL
RIGHT_WALL
PREDICTED
DEAD_RECKONING
SAFE_STOP
```

동작:

1. 진입 시 GPS 위치, 방향, 좌우 목표 벽 거리와 IMU 바이어스를 저장한다.
2. 좌우 벽의 거리와 각도에서 횡오차·진행각 오차를 계산한다.
3. 양쪽 벽이 유효하면 신뢰도 가중 평균을 사용한다.
4. 한쪽 벽만 유효하면 진입 시 저장한 목표 벽 거리를 사용한다.
5. 벽이 잠시 사라지면 마지막 조향값을 유지하지 않고 마지막 벽 상태를 IMU로 예측한다.
6. 예측한 상태로 매 주기 새로운 상대 로컬패스를 만든다.
7. 장시간 벽이 사라지면 감속 후 안전 정지한다.
8. 출구에서 GPS가 연속 정상이고 예상 위치와 일치하면 절대경로 복귀를 요청한다.

`TunnelInput::left_wall_points`, `right_wall_points`는 차량 좌표계의 벽 포인트다.

```text
x: 차량 전방
y: 차량 좌측
```

현재 `/LiDAR/wall_dist`의 끝점만 연결할 수도 있지만, 근거리·원거리 일관성과 robust fit을 충분히 활용하려면 필터링된 벽 포인트를 전달하는 것이 권장된다.

주요 튜닝값은 모두 `TunnelConfig`에 모여 있다.

- 벽 fit 최소 포인트 수와 잔차
- 근거리·원거리 범위
- 센서 timeout
- 추측항법 허용시간
- 목표속도
- 상대경로 길이
- GPS 복구 연속 샘플 수

### `static_traffic_mission.hpp/.cpp`

`StaticTrafficMission::update()`에 `StaticTrafficInput`을 전달하면 Frenet 후보 경로와 목표속도를 반환한다.

입력 장애물은 기준경로의 Frenet 좌표로 전달한다.

```text
s: 기준경로를 따라간 거리
d: 기준경로 좌측 방향 거리
```

주요 처리:

1. 장애물을 시간과 거리로 연관하여 추적한다.
2. 설정된 횟수 이상 검출된 장애물만 확정한다.
3. 차량 크기, 제어오차, 인지오차, 안전여유로 장애물을 확장한다.
4. 도로 좌우 경계 안에서 여러 횡 오프셋을 생성한다.
5. 조기 회피 거리와 복귀 길이가 다른 5차 다항식 후보를 생성한다.
6. 확장 장애물 충돌, 경계, 곡률을 검사한다.
7. 유효 후보 중 오프셋·곡률·길이·방향변경 비용이 가장 작은 경로를 선택한다.
8. 유효 경로가 없으면 `SAFE_STOP`을 반환한다.

신호등 정책:

```text
긴급 충돌 위험
> RED/YELLOW 정지
> 장애물 회피
> 일반 경로
```

- 신호 데이터가 오래되면 `UNKNOWN`
- 녹색은 연속 검출 후 확정
- 정지선 근처의 `UNKNOWN`은 정지
- 회피 경로도 적색 신호의 정지선을 넘지 않음

모든 차량·도로·안전 파라미터는 `StaticTrafficConfig`에서 수정할 수 있다.

### `mission_2026_test.cpp`

외부 프레임워크 없이 실행되는 합성 회귀시험이다.

- 양쪽 벽 추종
- 벽 순간·중기·장기 손실 fallback
- GPS 출구 복구
- 정적 장애물 회피 후보
- 적색 및 오래된 신호의 안전 정지
- 회피 공간이 없는 좁은 도로의 안전 정지

### `mission_2026_ros1_adapter.hpp/.cpp`

순수 C++ 미션 입력과 현재 프로젝트의 ROS1 토픽·`Path` 자료형 사이를 변환한다.

- 모든 센서 입력을 timestamp와 함께 캐시
- `/LiDAR/wall_dist`의 두 선분을 좌·우 벽 포인트로 변환
- 차량 기준 장애물을 UTM 및 Frenet `s,d`로 변환
- Frenet 회피 경로를 기존 절대 로컬패스 배열로 변환
- 센서가 오래됐거나 경로 생성에 실패하면 정지 경로 발행

`Planning::chooseFunc()`는 미션 번호 23, 14, 31에서 이 어댑터를 실행하고 결과를 공용 `Path`와 `Control` 객체에 반영한다. 따라서 `planning_node.cpp`는 다른 미션과 동일한 타이머·발행 흐름을 유지한다. 미션 14에서는 정적 장애물 정책만 적용하고, 미션 31에서만 신호등·정지선 정책을 함께 적용한다. 번호는 ROS 파라미터로 변경할 수 있다.

### `mission_2026_ros1_params.yaml`

미션 번호, 센서 timeout, 터널 치수, 차량 크기, 속도 및 도로 경계의 예시 설정이다. launch 파일의 `planning_node` 태그 내부에서 로드하면 private 파라미터로 적용된다.

```xml
<rosparam command="load"
          file="$(find planning_pkg_2025)/src/mission_2026/mission_2026_ros1_params.yaml"/>
```

## ROS1 입출력 연결

### GPS 터널

입력 변환:

```text
/imu                       → imu_yaw_rate_rps
/ERP/serial_data[3]        → vehicle_speed_mps
/fix                       → GpsSample
LiDAR 벽 포인트            → left/right_wall_points
/LiDAR/tunnel_end          → exit_hint
```

출력 변환:

```text
relative_path.toPlanningPositionArray()
    → /Planning/local_path

relative_path.yawArray()
    → /Planning/path_yaw

relative_path.curvatureArray()
    → /Planning/curvature

target_speed_mps
    → /Planning/target_velocity
```

### 정적 장애물·신호등

```text
/Local/utm                 → 차량 UTM 위치
/Local/heading             → 차량 heading
/LiDAR/object_cen          → 차량 기준 [x, y] 장애물 중심 배열
/Vision/traffic_sign       → [red, yellow, left, green]
/Vision/stopline           → 정지선 UTM 위치(선택 입력)
param.json stop_line_list  → 인지 정지선이 없을 때 사용하는 지도 정지선
현재 global Path           → Frenet 기준경로
```

출력은 터널과 동일한 `/Planning/local_path`, `/Planning/path_yaw`,
`/Planning/curvature`, `/Planning/target_velocity`를 사용한다.

현재 장애물 메시지에는 크기가 없으므로 YAML의 기본 폭·길이를 사용한다.
`mission_2026/static/obstacle_topic`으로 다른 `Float64MultiArray` 장애물
토픽을 선택할 수 있다. 인지 패키지에서 실제 폭·길이를 제공하게 되면
`obstacleCallback()`의 입력 변환을 확장해야 한다.

### 동적 장애물 (미션 24)

`dynamic_obstacle_stop.hpp/.cpp`는 제공된 2026 동적 장애물 판단기를 현재
ROS1 `Planning` 클래스에 연결한다. 미션 24에서 `doMission()`을 실행하며,
기준 속도는 `data/param.json`의 `mission.dynamic_obstacle.target_velocity`로
설정한다. 신호등 우선 정지 함수는 분리된 상태로 유지해 동적 장애물 미션에는
자동 적용하지 않는다.

```text
LiDAR_standard.cpp
    ├─ /LiDAR/car_dis                 (Float32, 경로 중심부 최근 거리)
    └─ /LiDAR/dynamic_obstacle_pos    (Float64MultiArray, [x1,y1,...])
                                      ↓
Planning Lidar cache + frame generation
                                      ↓
DynamicObstacleStop (TTC/CPA/정지 판별/회피 경로)
                                      ↓
/Planning/local_path + /Planning/target_velocity
```

인지되지 않은 프레임에서도 빈 좌표 배열과 안전거리 25 m를 함께 발행하므로,
직전 장애물이 Planning과 Plotting에 남지 않는다. Plotting에서는 최근 0.5초
이내의 동적 장애물을 자홍색 `X`로 표시한다.

### 고속도로 차선변경 (미션 60 → 100)

`highway_lane_change.hpp`는 `highway_test` 맵의 `1-0`부터 `1-3`까지를
각각 4·3·2·1차로로 사용한다. 미션 60 진입 구간 뒤 미션 100에서 좌측 차량
트랙과 현재 차로 선행차 TTC를 확인하고, 안전 조건이 유지되면 5차 다항식
경로로 한 차로씩 이동한다.

```bash
roslaunch morai_bringup morai_system.launch \
  map_name:=highway_test \
  mission_list:="[60, 100]" \
  use_plotting:=true
```

고속도로 미션용 주요 토픽은 다음과 같다.

```text
/LiDAR/mission100_left_tracks
/LiDAR/mission100_current_front_tracks
/LiDAR/car_front_car_dis
/Planning/mission100_current_lane
/Planning/plot_global_path_point
```

## 독립 빌드

이 폴더 안의 `CMakeLists.txt`는 핵심 로직과 테스트만 독립적으로 빌드한다.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

상위 `Planning_PKG_2025/CMakeLists.txt`에는 핵심 소스와 ROS1 어댑터가 `planning_node` 빌드 대상으로 추가되어 있다.

## 개발 원칙

- 시간에 민감한 입력은 반드시 `stamp`와 timeout을 사용한다.
- 마지막 센서값을 무기한 정상값으로 사용하지 않는다.
- 유효한 경로가 없으면 무리하게 회피하지 않고 정지한다.
- 파라미터를 코드 중간에 하드코딩하지 않고 Config 구조체에 둔다.
- 알고리즘 코어와 ROS 입출력을 분리한다.
- 실제 주행 전 합성시험 → rosbag → 시뮬레이터 순서로 검증한다.
