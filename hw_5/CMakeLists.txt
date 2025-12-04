cmake_minimum_required(VERSION 3.8)
project(hw_5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetWorkspaceBounds.srv"
)

# Install Python package directory
install(DIRECTORY hw_5/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables with RENAME to remove .py extension
install(PROGRAMS hw_5/motor_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_control
)
install(PROGRAMS hw_5/velocity_mapping.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME velocity_mapping
)
install(PROGRAMS hw_5/hw5.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hw5
)
install(PROGRAMS hw_5/camera_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_tf
)
install(PROGRAMS hw_5/ekf_slam_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ekf_slam
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/configs/
)

ament_package()
