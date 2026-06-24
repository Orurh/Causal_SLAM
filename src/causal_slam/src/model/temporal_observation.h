#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "coverage/imu_coverage_analyzer.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "model/point_time_diagnostics.h"
#include "telemetry/stream_timing_diagnostic.h"
#include "transform/transform_age_analyzer.h"

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
