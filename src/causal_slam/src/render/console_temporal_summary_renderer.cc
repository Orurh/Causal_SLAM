#include "render/console_temporal_summary_renderer.h"

#include <sstream>
#include <string>

namespace causal_slam::render {
namespace {

std::string CoverageStatus(
    const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  if (!snapshot.has_imu_coverage) {
    return "not_available";
  }

  return causal_slam::coverage::ToString(snapshot.imu_coverage.health);
}

}  // namespace

std::string ConsoleTemporalSummaryRenderer::Render(
    const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) const {
  std::ostringstream out;

  out << "Temporal Health: "
      << causal_slam::diagnostics::ToString(snapshot.overall_status) << '\n';

  out << "LiDAR scan window:\n"
      << "  source: "
      << causal_slam::lidar::ToString(snapshot.lidar_scan_window.source) << '\n'
      << "  confidence: "
      << causal_slam::lidar::ToString(snapshot.lidar_scan_window.confidence)
      << '\n'
      << "  duration_ms: " << snapshot.lidar_scan_window.duration_ms << '\n'
      << "  reason: " << snapshot.lidar_scan_window.reason << '\n';

  out << "IMU coverage:\n"
      << "  status: " << CoverageStatus(snapshot) << '\n'
      << "  samples_in_window: " << snapshot.imu_coverage.imu_count_in_window
      << '\n'
      << "  coverage_ratio: " << snapshot.imu_coverage.coverage_ratio << '\n'
      << "  max_gap_inside_ms: " << snapshot.imu_coverage.max_gap_inside_ms
      << '\n';

  out << "Streams:\n"
      << "  IMU: "
      << causal_slam::telemetry::ToString(snapshot.imu_timing.health)
      << " | period_ms=" << snapshot.imu_timing.last_period_ms
      << " | jitter_ms=" << snapshot.imu_timing.last_jitter_ms
      << " | gaps=" << snapshot.imu_timing.window_gap_count
      << " | reordered=" << snapshot.imu_timing.window_reordered_count << '\n'
      << "  LiDAR: "
      << causal_slam::telemetry::ToString(snapshot.lidar_timing.health)
      << " | period_ms=" << snapshot.lidar_timing.last_period_ms
      << " | jitter_ms=" << snapshot.lidar_timing.last_jitter_ms
      << " | gaps=" << snapshot.lidar_timing.window_gap_count
      << " | reordered=" << snapshot.lidar_timing.window_reordered_count
      << '\n';

  if (!snapshot.lidar_point_time.inspection_reason.empty()) {
    out << "LiDAR point time:\n"
        << "  candidate: "
        << (snapshot.lidar_point_time.has_time_candidate ? "true" : "false")
        << '\n'
        << "  supported: "
        << (snapshot.lidar_point_time.has_supported_time_field ? "true" : "false")
        << '\n'
        << "  field: " << snapshot.lidar_point_time.field_name << '\n'
        << "  datatype: " << snapshot.lidar_point_time.field_datatype << '\n'
        << "  reason: " << snapshot.lidar_point_time.inspection_reason << '\n';
  }

  if (snapshot.issues.empty()) {
    out << "Issues:\n"
        << "  none\n";
    return out.str();
  }

  out << "Issues:\n";

  for (const auto& issue : snapshot.issues) {
    out << "  [" << causal_slam::diagnostics::ToString(issue.severity) << "] "
        << issue.title << '\n'
        << "    Why: " << issue.explanation << '\n'
        << "    Evidence: " << issue.evidence << '\n'
        << "    Action: " << issue.suggested_action << '\n';
  }

  return out.str();
}

}  // namespace causal_slam::render