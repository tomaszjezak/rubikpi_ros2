cmake_minimum_required(VERSION 3.8)
project(grid_coverage_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(grid_map STATIC
  src/grid_map.cpp
)

set_property(TARGET grid_map PROPERTY POSITION_INDEPENDENT_CODE ON)

ament_package()
