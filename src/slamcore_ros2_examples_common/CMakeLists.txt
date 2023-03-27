cmake_minimum_required(VERSION 3.5)

# init config ------------------------------------------------------------------
if(NOT BUILD_VERSION)
  set(BUILD_VERSION 0.1.0)
endif()
project(
  slamcore_ros2_examples_common
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
find_package(nav_msgs REQUIRED)

# testing ----------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install ----------------------------------------------------------------------
install(DIRECTORY config descriptions launch meshes rviz
        DESTINATION share/${PROJECT_NAME})
install(PROGRAMS python/scripts/static_transform_publisher_from_file.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME static_transform_publisher_from_file)

# install python module
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

ament_package()
