cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

# Default to C++14 or C++17 depending on Foxy standards
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings to ensure code quality and safety standards
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)       # Uncomment if adding C++ nodes later
# find_package(gazebo_ros REQUIRED)   # Uncomment if linking against Gazebo C++ API


# 1. Install directories: Launch files, URDF models, maps, configs, and worlds.
# These assets are installed to: install/share/my_robot/
install(
  DIRECTORY launch urdf maps config worlds meshes scripts
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install Python executables/scripts.
# These are installed to: install/lib/my_robot/
# Users can invoke them via: ros2 run my_robot <script_name>
install(
  PROGRAMS
    scripts/robot_teleop.py
    scripts/imu_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude copyright/license checks for rapid development; enable in production.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()