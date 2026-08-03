cmake_minimum_required(VERSION 3.0.2)
project(lidar)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  vision_msgs
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs vision_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})


add_executable(lidar_standard  src/LiDAR_standard.cpp)
target_link_libraries(lidar_standard ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(lidar_mission       src/LiDAR_mission.cpp)
target_link_libraries(lidar_mission ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS lidar_standard lidar_mission
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
