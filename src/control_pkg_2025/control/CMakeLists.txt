cmake_minimum_required(VERSION 3.0.2)
project(control)

## 컴파일러 설정 (C++11 이상 필요, Noetic은 14 권장)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## 의존성 패키지 찾아오기 (ROS1용)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  roslib
)

find_package(nlohmann_json 3.2.0 REQUIRED)

## Catkin 패키지 설정
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs roslib
)

## 인클루드 경로 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 실행 파일 생성
add_executable(car_control src/car_control.cpp)

## 라이브러리 링크 (ROS1 방식)
target_link_libraries(car_control
  ${catkin_LIBRARIES}
  nlohmann_json::nlohmann_json
)

## 의존성 타겟 설정 (메시지 생성 순서 보장)
add_dependencies(car_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 설치 설정
install(TARGETS car_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/control/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.json"
)
