cmake_minimum_required(VERSION 3.8)
project(prm_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPRMPath.srv"
)

# Install Python package directory
install(DIRECTORY prm_planner/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executable
install(PROGRAMS
  prm_planner/prm_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prm_planner_node
)

ament_package()
