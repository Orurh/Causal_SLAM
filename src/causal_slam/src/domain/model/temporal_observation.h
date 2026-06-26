#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "domain/sensors/imu/imu_coverage_analyzer.h"
#include "domain/sensors/lidar/lidar_scan_window_estimator.h"
#include "domain/sensors/transform/transform_age_analyzer.h"
#include "domain/telemetry/stream_timing_diagnostic.h"
#include "point_time_diagnostics.h"

namespace causal_slam::model {

struct TemporalObservation {
  bool has_lidar_scan{false};
  std::int64_t latest_lidar_header_stamp_ns{0};
  std::string latest_lidar_frame_id;

  std::vector<causal_slam::telemetry::StreamTimingDiagnostic> streams;

  std::optional<causal_slam::coverage::ImuCoverageSummary> imu_coverage;
  std::optional<causal_slam::lidar::LidarScanWindowEstimate> lidar_scan_window;
  std::optional<PointTimeDiagnostics> lidar_point_time;
  std::vector<causal_slam::transform::TransformAgeSummary> transform_ages;

  std::size_t imu_buffer_size{0};
};

}  // namespace causal_slam::model
