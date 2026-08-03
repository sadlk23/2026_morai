cmake_minimum_required(VERSION 3.0.2) # (ros1용)
project(lidar_lane)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (ros1용: ament → catkin)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs visualization_msgs pcl_conversions cv_bridge) # (ros1용)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# ros2에서 쓰던 ament 목록은 catkin 컴포넌트로 대체 (ros1용)
set(dependencies
  roscpp            # (ros1용)
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  cv_bridge
)

# catkin 패키지 선언 (ros1용)
catkin_package() # (ros1용)

# Link libraries
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}   # (ros1용)
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Build the lidar_lane executable
add_executable(lidar_lane src/lidar_lane.cpp)
target_link_libraries(lidar_lane
  ${catkin_LIBRARIES}      # (ros1용)
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Install the executables (ros1용 경로 변수로 교체)
install(TARGETS lidar_lane
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # (ros1용)
)

# Linting and testing dependencies (ros1에서는 ament_lint 미사용)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()
# ament_package()  # (ros1용: 제거)