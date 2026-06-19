cmake_minimum_required(VERSION 3.8)
project(causal_slam)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(temporal_monitor_node
  src/nodes/temporal_monitor_main.cc
  src/nodes/temporal_monitor_node.cc
  src/telemetry/imu_timing_tracker.cc
)

target_include_directories(temporal_monitor_node
  PRIVATE
    src
)

add_executable(fake_imu_publisher_node
  src/nodes/fake_imu_publisher_node.cc
)

ament_target_dependencies(fake_imu_publisher_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(temporal_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
)

install(TARGETS 
  temporal_monitor_node
  fake_imu_publisher_node 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
