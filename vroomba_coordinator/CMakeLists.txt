cmake_minimum_required(VERSION 3.8)
project(vroomba_coordinator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_coverage_base REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetNextWaypoint.srv"
)

# Include directories
include_directories(include)

# Create executable
add_executable(vroomba_coordinator_node
  src/vroomba_coordinator.cpp
)

# Target dependencies
ament_target_dependencies(vroomba_coordinator_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  grid_coverage_base
  rosidl_default_runtime
)

# Link libraries
target_link_libraries(vroomba_coordinator_node
  grid_coverage_base_lib
)

# Link service types
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vroomba_coordinator_node ${cpp_typesupport_target})

# Install targets
install(TARGETS
  vroomba_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.hpp"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install service files
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

