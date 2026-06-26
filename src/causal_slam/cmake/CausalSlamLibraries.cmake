set(CAUSAL_SLAM_DOMAIN_SOURCES
  src/domain/telemetry/stream_timing_tracker.cc
  src/domain/sensors/transform/transform_age_analyzer.cc

  src/domain/sensors/lidar/lidar_scan_timing.cc
  src/domain/sensors/lidar/lidar_scan_window_estimator.cc

  src/domain/sensors/pointcloud/point_cloud2_field_inspector.cc
  src/domain/sensors/pointcloud/point_cloud2_time_field_extractor.cc

  src/domain/sensors/imu/imu_coverage_analyzer.cc
  src/domain/sensors/imu/imu_sample_buffer.cc

  src/domain/diagnostics/temporal_diagnostics.cc
  src/domain/diagnostics/temporal_fault_reason_formatter.cc

  src/domain/policy/lidar_cloud_gate.cc

  src/domain/statistics/temporal_statistics.cc
)

add_library(causal_slam_domain STATIC
  ${CAUSAL_SLAM_DOMAIN_SOURCES}
)

target_include_directories(causal_slam_domain
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(CAUSAL_SLAM_APPLICATION_SOURCES
  src/application/temporal_monitor/temporal_monitor_pipeline.cc
  src/application/temporal_monitor/temporal_monitor_runtime_defaults.cc
)

add_library(causal_slam_application STATIC
  ${CAUSAL_SLAM_APPLICATION_SOURCES}
)

target_include_directories(causal_slam_application
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(causal_slam_application
    causal_slam_domain)

set(CAUSAL_SLAM_PRESENTATION_SOURCES
  src/presentation/report/temporal_report_builder.cc

  src/presentation/render/console_temporal_summary_renderer.cc
  src/presentation/render/html_temporal_summary_renderer.cc
  src/presentation/render/map_update_decision_json_renderer.cc
)

add_library(causal_slam_presentation STATIC
  ${CAUSAL_SLAM_PRESENTATION_SOURCES}
)

target_include_directories(causal_slam_presentation
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(causal_slam_presentation
    causal_slam_domain)

add_library(causal_slam_platform STATIC
  src/platform/atomic_file_writer.cc
)

target_include_directories(causal_slam_platform
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(CAUSAL_SLAM_ROS_ADAPTER_SOURCES
  src/adapters/ros2/point_cloud2_conversions.cc
  src/adapters/ros2/transform_lookup_adapter.cc
)

add_library(causal_slam_ros_adapters STATIC
  ${CAUSAL_SLAM_ROS_ADAPTER_SOURCES}
)

target_include_directories(causal_slam_ros_adapters
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(causal_slam_ros_adapters
    causal_slam_domain)

ament_target_dependencies(causal_slam_ros_adapters
  rclcpp
  sensor_msgs
  builtin_interfaces
  geometry_msgs
  tf2
  tf2_ros
)
