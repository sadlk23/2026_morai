cmake_minimum_required(VERSION 3.16)
project(mission_2026_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mission_2026_core
    mission_math.cpp
    gps_shadow_tunnel.cpp
    static_traffic_mission.cpp
)

target_include_directories(mission_2026_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

if(MSVC)
    target_compile_options(mission_2026_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(mission_2026_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(mission_2026_test
    mission_2026_test.cpp
)
target_link_libraries(mission_2026_test PRIVATE mission_2026_core)

enable_testing()
add_test(NAME mission_2026_core_test COMMAND mission_2026_test)

