cmake_minimum_required(VERSION 3.5)
project(rosmower)

# C standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core
find_package(ament_cmake REQUIRED)

# ROS 2 deps for the Python nodes
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Install resources (your existing assets)
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/hoverboard_bridge_node.py
  scripts/lidar_scan_guard.py
  scripts/imu_bridge.py
  scripts/rplidar_motor_control.py
  scripts/battery_splitter.py
  scripts/relay_control_node.py
  scripts/vl53_bridge.py
  scripts/tof_to_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

# Build and install C++ rplidar motor control node
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)



install(TARGETS

  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# <<< MUST BE LAST >>>
ament_package()
