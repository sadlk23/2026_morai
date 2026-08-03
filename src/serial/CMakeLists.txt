cmake_minimum_required(VERSION 3.0.2)
project(serial)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  morai_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs morai_msgs
)

catkin_install_python(PROGRAMS
  scripts/morai_udp_main.py
  scripts/morai_udp_ai.py
  scripts/collision_udp_to_ros.py
  scripts/enu_to_utm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(FILES params.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
