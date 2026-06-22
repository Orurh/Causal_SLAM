#include "render/console_temporal_summary_renderer.h"

#include "policy/map_update_decision.h"

#include <sstream>
#include <string>
#include <string_view>

namespace causal_slam::render {
namespace {

std::string CoverageStatus(
    const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  if (!snapshot.observation.imu_coverage.has_value()) {
    return "not_available";
  }

  return causal_slam::coverage::ToString(snapshot.observation.imu_coverage->health);
}

void RenderNumericStats(std::ostringstream& out, std::string_view indent, std::string_view name,
                        const causal_slam::statistics::NumericStats& stats) {
  out << indent << name << ": ";

  if (stats.count == 0) {
    out << "n/a\n";
    return;
  }

  out << "count=" << stats.count << " | mean=" << stats.mean << " | median=" << stats.median << " | p95=" << stats.p95
      << " | max=" << stats.max << '\n';
}

void RenderStreamTimingStatistics(std::ostringstream& out, const causal_slam::statistics::StreamTimingStatistics& stream) {
  out << "    " << causal_slam::telemetry::ToString(stream.id) << ":\n";
  RenderNumericStats(out, "      ", "delay_ms", stream.delay_ms);
  RenderNumericStats(out, "      ", "period_ms", stream.period_ms);
  RenderNumericStats(out, "      ", "jitter_ms", stream.jitter_ms);
}

void RenderWindowStatistics(std::ostringstream& out, const char* title, const causal_slam::statistics::TemporalWindowStatistics& stats) {
  out << title << ":\n";
  out << "  samples: " << stats.sample_count << '\n';

  out << "  health:"
      << " OK=" << stats.health.ok_count << " WARNING=" << stats.health.warning_count << " DEGRADED=" << stats.health.degraded_count
      << " INVALID=" << stats.health.invalid_count
      << '\n';

  out << "  scan_window_sources:"
      << " point_time_field=" << stats.scan_window_sources.point_time_field_count
      << " measured_header_period=" << stats.scan_window_sources.measured_header_period_count
      << " assumed_fixed_duration=" << stats.scan_window_sources.assumed_fixed_duration_count
      << " driver_metadata=" << stats.scan_window_sources.driver_metadata_count << '\n';

  out << "  scan_window_confidence:"
      << " HIGH=" << stats.scan_window_confidence.high_count << " MEDIUM=" << stats.scan_window_confidence.medium_count
      << " LOW=" << stats.scan_window_confidence.low_count << '\n';

  out << "  streams:\n";
  if (stats.streams.empty()) {
    out << "    none\n";
  } else {
    for (const auto& stream : stats.streams) {
      RenderStreamTimingStatistics(out, stream);
    }
  }

  RenderNumericStats(out, "  ", "imu_coverage_ratio", stats.imu_coverage_ratio);
  RenderNumericStats(out, "  ", "imu_samples_in_window", stats.imu_samples_in_window);
  RenderNumericStats(out, "  ", "imu_max_gap_inside_ms", stats.imu_max_gap_inside_ms);
}

}  // namespace

std::string ConsoleTemporalSummaryRenderer::Render(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) const {
  std::ostringstream out;

  out << "Temporal Health: "
      << causal_slam::telemetry::ToString(snapshot.overall_status) << '\n';

  out << "Map update:\n"
      << "  allowed: "
      << (snapshot.map_update_decision.map_update_allowed ? "true" : "false")
      << '\n'
      << "  reason: "
      << causal_slam::policy::ToString(snapshot.map_update_decision.reason)
      << '\n';

  out << "LiDAR scan window:\n";
  if (!snapshot.observation.lidar_scan_window.has_value()) {
    out << "  status: not_available\n"
        << "  reason: no_lidar_scan_received_yet\n";
  } else {
    const auto& scan_window = *snapshot.observation.lidar_scan_window;
    out << "  source: " << causal_slam::lidar::ToString(scan_window.source)
        << '\n'
        << "  confidence: "
        << causal_slam::lidar::ToString(scan_window.confidence) << '\n'
        << "  duration_ms: " << scan_window.duration_ms << '\n'
        << "  reason: " << scan_window.reason << '\n';
  }

  out << "IMU coverage:\n";
  if (!snapshot.observation.imu_coverage.has_value()) {
    out << "  status: not_available\n"
        << "  reason: no_lidar_scan_received_yet\n";
  } else {
    const auto& coverage = *snapshot.observation.imu_coverage;
    out << "  status: " << CoverageStatus(snapshot) << '\n'
        << "  samples_in_window: " << coverage.imu_count_in_window << '\n'
        << "  coverage_ratio: " << coverage.coverage_ratio << '\n'
        << "  max_gap_inside_ms: " << coverage.max_gap_inside_ms << '\n';
  }

  out << "Streams:\n";
  if (snapshot.observation.streams.empty()) {
    out << "  none\n";
  } else {
    for (const auto& stream : snapshot.observation.streams) {
      const auto& summary = stream.timing;

      out << "  " << causal_slam::telemetry::ToString(stream.id) << ": " << causal_slam::telemetry::ToString(summary.health) << " | period_ms=" << summary.last_period_ms
          << " | jitter_ms=" << summary.last_jitter_ms << " | gaps=" << summary.window_gap_count
          << " | reordered=" << summary.window_reordered_count << '\n';
    }
  }

  if (snapshot.observation.lidar_point_time.has_value() &&
      !snapshot.observation.lidar_point_time->inspection_reason.empty()) {
    const auto& point_time = *snapshot.observation.lidar_point_time;

    out << "LiDAR point time:\n"
        << "  candidate: " << (point_time.has_time_candidate ? "true" : "false")
        << '\n'
        << "  supported: "
        << (point_time.has_supported_time_field ? "true" : "false") << '\n'
        << "  field: " << point_time.field_name << '\n'
        << "  datatype: " << point_time.field_datatype << '\n'
        << "  reason: " << point_time.inspection_reason << '\n';
  }

  if (snapshot.issues.empty()) {
    out << "Issues:\n"
        << "  none\n";
    return out.str();
  }

  out << "Issues:\n";

  for (const auto& issue : snapshot.issues) {
    out << "  [" << causal_slam::diagnostics::ToString(issue.severity) << "] "
        << issue.title << " | reason="
        << causal_slam::diagnostics::ToString(issue.reason) << '\n'
        << "    Why: " << issue.explanation << '\n'
        << "    Evidence: " << issue.evidence << '\n'
        << "    Action: " << issue.suggested_action << '\n';
  }

  return out.str();
}

std::string ConsoleTemporalSummaryRenderer::RenderStatistics(const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const {
  std::ostringstream out;

  out << "Temporal Statistics (historical windows)\n";

  RenderWindowStatistics(out, "Last 10s", snapshot.short_window);
  RenderWindowStatistics(out, "Last 60s", snapshot.medium_window);
  RenderWindowStatistics(out, "Session", snapshot.session);

  return out.str();
}

}  // namespace causal_slam::render