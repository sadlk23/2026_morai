cmake_minimum_required(VERSION 3.0.2)
project(morai)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation            # ★ 추가
)

## ★ 메시지 파일 추가
add_message_files(
  FILES
  ObjectStatus.msg
  CollisionData.msg
)

## ★ 메시지 생성
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime   # ★ message_runtime 추가
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/gps_to_utm.py
  scripts/morai_imu.py
  scripts/morai_gps.py
  scripts/morai_collisionData.py   # 이 노드가 새 메시지를 publish/import 함
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
