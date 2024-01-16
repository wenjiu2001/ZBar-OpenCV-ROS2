cmake_minimum_required(VERSION 3.5)
project(zbar_opencv_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

# Executable
add_executable(zbar_opencv_ros2 src/zbar_opencv_ros2.cpp)

# Include directories
# Include only if there are headers in the 'include' directory
include_directories(include)

# Ament target dependencies
ament_target_dependencies(zbar_opencv_ros2
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Link libraries
target_link_libraries(zbar_opencv_ros2
  ${OpenCV_LIBRARIES}
  PkgConfig::ZBAR
)

# Installation
install(TARGETS
  zbar_opencv_ros2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Test
option(BUILD_TESTS "Build test targets" ON)
if(BUILD_TESTS AND BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_package()
