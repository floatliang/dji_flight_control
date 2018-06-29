cmake_minimum_required(VERSION 2.8.3)
project(wifi_ekf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    )

catkin_package(
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )
add_executable(wifi_ekf
    src/wifi_ekf_node.cpp
    src/wifi_ekf.cpp
    )

target_link_libraries(wifi_ekf ${catkin_LIBRARIES})
