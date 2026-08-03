# ROS1 2026 판단 미션 1~3 실행 워크스페이스

정적 장애물·GPS 음영 터널, 동적 장애물, 고속도로 미션만 실행하도록
정리한 ROS1 Noetic catkin 워크스페이스다.

## 지원 미션

| 번호 | 기능 |
|---:|---|
| 14 | 정적 장애물 |
| 23 | GPS 음영 터널 |
| 24 | 동적 장애물 |
| 31 | 정적 장애물·신호 |
| 60 | 고속도로 진입 |
| 100 | 고속도로 4→3→2→1차로 변경 |

Planning에서 그 외 미션 번호가 들어오면 목표속도 0으로 안전 정지한다.

## 포함 내부 패키지

- `planning_pkg_2025`
- `lidar`, `lidar_lane`, `perception`
- `morai`, `morai_msgs`, `serial`
- `control`
- `morai_bringup`
- `plotting_pkg` (선택 실행)

외부 ROS/PCL/Velodyne 시스템 의존성은 Ubuntu에 설치되어 있어야 한다.

## 권장 환경

- Ubuntu 20.04
- ROS1 Noetic
- C++20을 지원하는 GCC
- MORAI Simulator

주요 시스템 의존성:

```bash
sudo apt update
sudo apt install \
  ros-noetic-desktop-full \
  ros-noetic-velodyne-pointcloud \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-vision-msgs \
  ros-noetic-cv-bridge \
  libcgal-dev libeigen3-dev nlohmann-json3-dev \
  python3-rosdep python3-pyproj python3-matplotlib python3-tk
```

## 빌드

```bash
cd ~/Morai_WS_2026_Mission_1-3_ROS1
chmod +x build_workspace.sh
./build_workspace.sh
source devel/setup.bash
```

또는 직접 빌드한다.

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 실행

고속도로 미션:

```bash
roslaunch morai_bringup mission_3_highway.launch
```

정적 장애물 → GPS 음영 터널 → 동적 장애물 → 정적·신호 통합 프리셋:

```bash
roslaunch morai_bringup mission_1_2.launch
```

공통 launch에 맵과 미션 목록을 직접 지정할 수도 있다.

```bash
roslaunch morai_bringup morai_system.launch \
  map_name:=highway_test \
  mission_list:="[60, 100]"
```

## 주요 입력

```text
/lidar/velodyne_points
/fix
/imu
/Local/utm
/Local/heading
/ERP/serial_data
/Vision/traffic_sign
/Vision/stopline
```

## 주요 출력

```text
/Planning/local_path
/Planning/path_yaw
/Planning/curvature
/Planning/target_velocity
/Planning/mission
/Planning/mission100_current_lane
```

## 주의

- `map_morai_test1`의 통합 프리셋은 프로그램 연결 시험용이다. 실제 미션
  배치와 UTM 트리거 좌표는 사용하는 MORAI 맵에 맞게 확인해야 한다.
- 고속도로는 ZIP에 포함된 `highway_test` 맵과 `[60, 100]` 구성을 사용한다.
- 현재 Windows 작업 환경에는 ROS/catkin/C++ 컴파일러가 없어 실제 빌드는
  수행하지 못했으며, Ubuntu ROS1 환경에서 최종 빌드가 필요하다.
