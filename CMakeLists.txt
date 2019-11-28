
cmake_minimum_required(VERSION 3.14)
project(nereid LANGUAGES C CXX)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)

set(ROS2_DIR /ros2-linux)

add_subdirectory(dependencies)
add_subdirectory(src)
