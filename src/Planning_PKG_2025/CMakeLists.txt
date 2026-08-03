cmake_minimum_required(VERSION 3.0.2)
project(planning_pkg_2025)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)  # C++20으로 설정
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# mode: release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# # mode: debug
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE Debug)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Eigen3
include_directories("/usr/include/eigen3")

# find dependencies (ROS1 catkin)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(nlohmann_json REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs roslib
  INCLUDE_DIRS include/${PROJECT_NAME} include/${PROJECT_NAME}/data_header include/${PROJECT_NAME}/data_process_header include/${PROJECT_NAME}/mission_header include/${PROJECT_NAME}/path_planning_header third_party
)

# Declare include directories
include_directories(
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/data_header
  include/${PROJECT_NAME}/data_process_header
  include/${PROJECT_NAME}/mission_header
  include/${PROJECT_NAME}/path_planning_header
  src/mission_2026
  third_party
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CGAL_INCLUDE_DIRS}
)

set(MISSION_2026_SOURCES
  src/mission_2026/mission_math.cpp
  src/mission_2026/gps_shadow_tunnel.cpp
  src/mission_2026/static_traffic_mission.cpp
  src/mission_2026/mission_2026_ros1_adapter.cpp
  src/mission_2026/dynamic_obstacle_stop.cpp
)

add_executable(planning_node
  src/planning_node.cpp
  src/path.cpp
  src/local.cpp
  src/polynomial.cpp
  src/optimalTrajectoryPlanner.cpp
  ${MISSION_2026_SOURCES}
)

target_link_libraries(planning_node
  ${catkin_LIBRARIES}
  ${CGAL_LIBRARIES}
  pthread
)

add_dependencies(planning_node ${catkin_EXPORTED_TARGETS})

install(TARGETS planning_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch data map config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(mission_2026_test
    src/mission_2026/mission_2026_test.cpp
    src/mission_2026/mission_math.cpp
    src/mission_2026/gps_shadow_tunnel.cpp
    src/mission_2026/static_traffic_mission.cpp
  )
  target_include_directories(mission_2026_test PRIVATE src/mission_2026)
  add_test(NAME mission_2026_core_test COMMAND mission_2026_test)
endif()

# ROS1: no ament testing block
