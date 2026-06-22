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

  src/pointcloud/point_cloud2_field_inspector.cc
  src/pointcloud/point_cloud2_time_field_extractor.cc

  src/coverage/imu_coverage_analyzer.cc
  src/coverage/imu_sample_buffer.cc

  src/diagnostics/temporal_diagnostics.cc
  src/diagnostics/temporal_fault_reason_formatter.cc
  src/statistics/temporal_statistics.cc

  src/pipeline/temporal_monitor_pipeline.cc

  src/render/console_temporal_summary_renderer.cc
)

add_library(causal_slam_core STATIC
  ${CAUSAL_SLAM_CORE_SOURCES}
)

target_include_directories(causal_slam_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(CAUSAL_SLAM_ROS_ADAPTER_SOURCES
  src/ros_adapters/point_cloud2_conversions.cc
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
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)

  function(add_causal_slam_core_gtest target_name)
    ament_add_gtest(${target_name} ${ARGN})

    if(TARGET ${target_name})
      target_link_libraries(${target_name}
        causal_slam_core
      )
    endif()
  endfunction()

  function(add_causal_slam_ros_adapter_gtest target_name)
    ament_add_gtest(${target_name} ${ARGN})

    if(TARGET ${target_name})
      target_link_libraries(${target_name}
        causal_slam_ros_adapters
      )

      ament_target_dependencies(${target_name}
        sensor_msgs
      )
    endif()
  endfunction()

  add_causal_slam_core_gtest(stream_timing_tracker_test
    tests/telemetry/stream_timing_tracker_test.cc
  )

  add_causal_slam_core_gtest(lidar_scan_timing_test
    tests/lidar/lidar_scan_timing_test.cc
  )

  add_causal_slam_core_gtest(lidar_scan_window_estimator_test
    tests/lidar/lidar_scan_window_estimator_test.cc
  )

  add_causal_slam_core_gtest(imu_coverage_analyzer_test
    tests/coverage/imu_coverage_analyzer_test.cc
  )

  add_causal_slam_core_gtest(imu_sample_buffer_test
    tests/coverage/imu_sample_buffer_test.cc
  )

  add_causal_slam_core_gtest(temporal_diagnostics_test
    tests/diagnostics/temporal_diagnostics_test.cc
  )

  add_causal_slam_core_gtest(temporal_fault_reason_formatter_test
    tests/diagnostics/temporal_fault_reason_formatter_test.cc
  )

  add_causal_slam_core_gtest(map_update_decision_test
    tests/policy/map_update_decision_test.cc
  )

  add_causal_slam_core_gtest(temporal_statistics_test
    tests/statistics/temporal_statistics_test.cc
  )

  add_causal_slam_core_gtest(point_cloud2_field_inspector_core_test
    tests/pointcloud/point_cloud2_field_inspector_test.cc
  )

  add_causal_slam_core_gtest(point_cloud2_time_field_extractor_core_test
    tests/pointcloud/point_cloud2_time_field_extractor_test.cc
  )

  add_causal_slam_core_gtest(temporal_monitor_pipeline_test
    tests/pipeline/temporal_monitor_pipeline_test.cc
  )

  add_test(
    NAME architecture_dependency_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_arch_deps.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_test(
    NAME ros1_compatibility_audit_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/audit_ros1_compatibility.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_test(
    NAME ros1_adapter_audit_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/audit_ros1_adapter.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()


ament_package()