cmake_minimum_required(VERSION 3.16)
project(causal_slam_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

# Default works when the whole causal_slam repository is placed under catkin_ws/src.
# Override with:
#   catkin_make -DCAUSAL_SLAM_ROOT=/absolute/path/to/causal_slam
set(CAUSAL_SLAM_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    CACHE PATH "Path to the causal_slam repository root")

catkin_package()

set(CAUSAL_SLAM_COMMON_SOURCES
  ${CAUSAL_SLAM_ROOT}/src/domain/telemetry/stream_timing_tracker.cc
  ${CAUSAL_SLAM_ROOT}/src/domain/sensors/transform/transform_age_analyzer.cc

  ${CAUSAL_SLAM_ROOT}/src/domain/sensors/lidar/lidar_scan_timing.cc
  ${CAUSAL_SLAM_ROOT}/src/domain/sensors/lidar/lidar_scan_window_estimator.cc

  ${CAUSAL_SLAM_ROOT}/src/domain/sensors/pointcloud/point_cloud2_field_inspector.cc
  ${CAUSAL_SLAM_ROOT}/src/domain/sensors/pointcloud/point_cloud2_time_field_extractor.cc

  ${CAUSAL_SLAM_ROOT}/src/domain/sensors/imu/imu_coverage_analyzer.cc
  ${CAUSAL_SLAM_ROOT}/src/domain/sensors/imu/imu_sample_buffer.cc

  ${CAUSAL_SLAM_ROOT}/src/domain/diagnostics/temporal_diagnostics.cc
  ${CAUSAL_SLAM_ROOT}/src/domain/diagnostics/temporal_fault_reason_formatter.cc
  ${CAUSAL_SLAM_ROOT}/src/domain/statistics/temporal_statistics.cc

  ${CAUSAL_SLAM_ROOT}/src/application/temporal_monitor/temporal_monitor_pipeline.cc

  ${CAUSAL_SLAM_ROOT}/src/presentation/render/console_temporal_summary_renderer.cc
)

add_library(causal_slam_ros1_core STATIC
  ${CAUSAL_SLAM_COMMON_SOURCES}
  src/point_cloud2_conversions.cpp
)

target_include_directories(causal_slam_ros1_core
  PUBLIC
    include
    ${CAUSAL_SLAM_ROOT}/src
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(causal_slam_ros1_core
  ${catkin_LIBRARIES}
)

add_executable(causal_slam_ros1_temporal_monitor_node
  src/temporal_monitor_node.cpp
)

target_include_directories(causal_slam_ros1_temporal_monitor_node
  PRIVATE
    include
    ${CAUSAL_SLAM_ROOT}/src
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(causal_slam_ros1_temporal_monitor_node
  causal_slam_ros1_core
  ${catkin_LIBRARIES}
)

install(TARGETS
  causal_slam_ros1_temporal_monitor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

