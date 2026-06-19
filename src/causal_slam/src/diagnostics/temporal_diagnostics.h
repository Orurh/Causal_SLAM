#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "coverage/imu_coverage_analyzer.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "telemetry/stream_timing_tracker.h"

namespace causal_slam::diagnostics {

enum class TemporalDiagnosticSeverity : std::uint8_t {
  kInfo,
  kWarning,
  kDegraded,
};

enum class TemporalHealthStatus : std::uint8_t {
  kOk,
  kWarning,
  kDegraded,
};

[[nodiscard]] const char* ToString(TemporalDiagnosticSeverity severity);
[[nodiscard]] const char* ToString(TemporalHealthStatus status);

struct TemporalDiagnosticIssue {
  TemporalDiagnosticSeverity severity{TemporalDiagnosticSeverity::kInfo};

  std::string title;
  std::string explanation;
  std::string evidence;
  std::string suggested_action;
};

struct PointTimeDiagnostics {
  bool has_time_candidate{false};
  bool has_supported_time_field{false};

  std::string field_name;
  std::string field_datatype;
  std::string field_role;

  std::string inspection_reason;

  bool extraction_attempted{false};
  bool extraction_used{false};
  std::string extraction_reason;
  std::string extraction_unit;
};

struct TemporalDiagnosticsInput {
  causal_slam::telemetry::TimingSummary imu_timing;
  causal_slam::telemetry::TimingSummary lidar_timing;

  bool has_imu_coverage{false};
  causal_slam::coverage::ImuCoverageSummary imu_coverage;

  causal_slam::lidar::LidarScanWindowEstimate lidar_scan_window;

  PointTimeDiagnostics lidar_point_time;

  std::size_t imu_buffer_size{0};
};

struct TemporalDiagnosticSnapshot {
  TemporalHealthStatus overall_status{TemporalHealthStatus::kOk};

  causal_slam::telemetry::TimingSummary imu_timing;
  causal_slam::telemetry::TimingSummary lidar_timing;

  bool has_imu_coverage{false};
  causal_slam::coverage::ImuCoverageSummary imu_coverage;

  causal_slam::lidar::LidarScanWindowEstimate lidar_scan_window;

  PointTimeDiagnostics lidar_point_time;

  std::size_t imu_buffer_size{0};

  std::vector<TemporalDiagnosticIssue> issues;
};

class TemporalDiagnosticsBuilder final {
 public:
  [[nodiscard]] TemporalDiagnosticSnapshot Build(
      const TemporalDiagnosticsInput& input) const;
};

}  // namespace causal_slam::diagnostics