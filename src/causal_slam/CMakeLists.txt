cmake_minimum_required(VERSION 3.8)
project(causal_slam)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(CAUSAL_SLAM_CORE_SOURCES
  src/telemetry/stream_timing_tracker.cc

  src/lidar/lidar_scan_timing.cc
  src/lidar/lidar_scan_window_estimator.cc

  src/coverage/imu_coverage_analyzer.cc
  src/coverage/imu_sample_buffer.cc
)

add_library(causal_slam_core STATIC
  ${CAUSAL_SLAM_CORE_SOURCES}
)

target_include_directories(causal_slam_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(CAUSAL_SLAM_ROS_ADAPTER_SOURCES
  src/ros_adapters/point_cloud2_field_inspector.cc
  src/ros_adapters/point_cloud2_time_field_extractor.cc
)

add_library(causal_slam_ros_adapters STATIC
  ${CAUSAL_SLAM_ROS_ADAPTER_SOURCES}
)

target_include_directories(causal_slam_ros_adapters
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(causal_slam_ros_adapters
  causal_slam_core
)

ament_target_dependencies(causal_slam_ros_adapters
  sensor_msgs
)


add_executable(temporal_monitor_node
  src/nodes/temporal_monitor_main.cc
  src/nodes/temporal_monitor_node.cc
)

target_link_libraries(temporal_monitor_node
  causal_slam_core
  causal_slam_ros_adapters
)

ament_target_dependencies(temporal_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
)

add_executable(fake_imu_publisher_node
  src/nodes/fake_imu_publisher_node.cc
)

ament_target_dependencies(fake_imu_publisher_node
  rclcpp
  sensor_msgs
)

add_executable(fake_lidar_publisher_node
  src/nodes/fake_lidar_publisher_node.cc
)

ament_target_dependencies(fake_lidar_publisher_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  temporal_monitor_node
  fake_imu_publisher_node
  fake_lidar_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  function(add_causal_slam_gtest target_name)
    ament_add_gtest(${target_name} ${ARGN})

    if(TARGET ${target_name})
      target_link_libraries(${target_name}
        causal_slam_core
      )
    endif()
  endfunction()

  add_causal_slam_gtest(stream_timing_tracker_test
    tests/telemetry/stream_timing_tracker_test.cc
  )

  add_causal_slam_gtest(lidar_scan_timing_test
    tests/lidar/lidar_scan_timing_test.cc
  )

  add_causal_slam_gtest(lidar_scan_window_estimator_test
    tests/lidar/lidar_scan_window_estimator_test.cc
  )

  add_causal_slam_gtest(imu_coverage_analyzer_test
    tests/coverage/imu_coverage_analyzer_test.cc
  )

  add_causal_slam_gtest(imu_sample_buffer_test
    tests/coverage/imu_sample_buffer_test.cc
  )

  ament_add_gtest(point_cloud2_field_inspector_test
    tests/ros_adapters/point_cloud2_field_inspector_test.cc
  )

  if(TARGET point_cloud2_field_inspector_test)
    target_link_libraries(point_cloud2_field_inspector_test
      causal_slam_ros_adapters
    )

    ament_target_dependencies(point_cloud2_field_inspector_test
      sensor_msgs
    )
  endif()

  ament_add_gtest(point_cloud2_time_field_extractor_test
    tests/ros_adapters/point_cloud2_time_field_extractor_test.cc
  )

  if(TARGET point_cloud2_time_field_extractor_test)
    target_link_libraries(point_cloud2_time_field_extractor_test
      causal_slam_ros_adapters
    )

    ament_target_dependencies(point_cloud2_time_field_extractor_test
      sensor_msgs
    )
  endif()


  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()