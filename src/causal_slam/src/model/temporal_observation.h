#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "coverage/imu_coverage_analyzer.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "model/point_time_diagnostics.h"
#include "telemetry/stream_timing_diagnostic.h"

namespace causal_slam::model {

struct TemporalObservation {
  std::vector<causal_slam::telemetry::StreamTimingDiagnostic> streams;

  std::optional<causal_slam::coverage::ImuCoverageSummary> imu_coverage;
  std::optional<causal_slam::lidar::LidarScanWindowEstimate> lidar_scan_window;
  std::optional<PointTimeDiagnostics> lidar_point_time;

  std::size_t imu_buffer_size{0};
};

}  // namespace causal_slam::model
