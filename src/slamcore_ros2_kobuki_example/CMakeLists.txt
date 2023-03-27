cmake_minimum_required(VERSION 3.5)

# init config ------------------------------------------------------------------
if(NOT BUILD_VERSION)
  set(BUILD_VERSION 0.1.0)
endif()
project(
  slamcore_ros2_kobuki_example
  VERSION ${BUILD_VERSION}
  LANGUAGES CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

# teleop node ------------------------------------------------------------------
add_executable(kobuki_teleop_joy src/kobuki_teleop_joy.cpp)
target_include_directories(kobuki_teleop_joy PRIVATE include)
ament_target_dependencies(kobuki_teleop_joy geometry_msgs rclcpp
                          kobuki_ros_interfaces std_msgs)
install(TARGETS kobuki_teleop_joy DESTINATION lib/${PROJECT_NAME})

# testing ----------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install ----------------------------------------------------------------------
install(DIRECTORY config descriptions launch meshes
        DESTINATION share/${PROJECT_NAME})

# Install Python executables ---------------------------------------------------
install(PROGRAMS python/scripts/kobuki_teleop_key.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME kobuki_teleop_key)

ament_package()
