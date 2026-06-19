#include "diagnostics/temporal_diagnostics.h"

#include <string>
#include <utility>

namespace causal_slam::diagnostics {
namespace {

TemporalHealthStatus MaxStatus(TemporalHealthStatus lhs, TemporalHealthStatus rhs) {
  if (lhs == TemporalHealthStatus::kDegraded || rhs == TemporalHealthStatus::kDegraded) {
    return TemporalHealthStatus::kDegraded;
  }

  if (lhs == TemporalHealthStatus::kWarning || rhs == TemporalHealthStatus::kWarning) {
    return TemporalHealthStatus::kWarning;
  }

  return TemporalHealthStatus::kOk;
}

TemporalHealthStatus StatusFromSeverity(TemporalDiagnosticSeverity severity) {
  switch (severity) {
    case TemporalDiagnosticSeverity::kInfo:
      return TemporalHealthStatus::kOk;
    case TemporalDiagnosticSeverity::kWarning:
      return TemporalHealthStatus::kWarning;
    case TemporalDiagnosticSeverity::kDegraded:
      return TemporalHealthStatus::kDegraded;
  }

  return TemporalHealthStatus::kDegraded;
}

TemporalDiagnosticSeverity SeverityFromTimingHealth(causal_slam::telemetry::TimingHealth health) {
  switch (health) {
    case causal_slam::telemetry::TimingHealth::kOk:
      return TemporalDiagnosticSeverity::kInfo;
    case causal_slam::telemetry::TimingHealth::kWarning:
      return TemporalDiagnosticSeverity::kWarning;
    case causal_slam::telemetry::TimingHealth::kDegraded:
      return TemporalDiagnosticSeverity::kDegraded;
  }

  return TemporalDiagnosticSeverity::kDegraded;
}

void AddTimingIssue(const char* stream_name, const causal_slam::telemetry::TimingSummary& summary,
                    std::vector<TemporalDiagnosticIssue>* issues) {
  if (summary.health == causal_slam::telemetry::TimingHealth::kOk) {
    return;
  }

  issues->push_back(TemporalDiagnosticIssue{
      .severity = SeverityFromTimingHealth(summary.health),
      .title = std::string(stream_name) + " timing is not stable",
      .explanation = "The message stream has temporal instability in the latest summary window.",
      .evidence = "reason=" + summary.reason + ", window_count=" + std::to_string(summary.window_count) + ", last_period_ms=" +
                  std::to_string(summary.last_period_ms) + ", window_max_jitter_ms=" + std::to_string(summary.window_max_jitter_ms) +
                  ", window_gap_count=" + std::to_string(summary.window_gap_count) +
                  ", window_reordered_count=" + std::to_string(summary.window_reordered_count),
      .suggested_action = "Check sensor driver timing, QoS, CPU load, transport latency, and timestamp source.",
  });
}

void AddIssueAndUpdateStatus(TemporalDiagnosticIssue issue, TemporalDiagnosticSnapshot* snapshot) {
  snapshot->overall_status = MaxStatus(snapshot->overall_status, StatusFromSeverity(issue.severity));

  snapshot->issues.push_back(std::move(issue));
}

}  // namespace

const char* ToString(TemporalDiagnosticSeverity severity) {
  switch (severity) {
    case TemporalDiagnosticSeverity::kInfo:
      return "INFO";
    case TemporalDiagnosticSeverity::kWarning:
      return "WARNING";
    case TemporalDiagnosticSeverity::kDegraded:
      return "DEGRADED";
  }

  return "UNKNOWN";
}

const char* ToString(TemporalHealthStatus status) {
  switch (status) {
    case TemporalHealthStatus::kOk:
      return "OK";
    case TemporalHealthStatus::kWarning:
      return "WARNING";
    case TemporalHealthStatus::kDegraded:
      return "DEGRADED";
  }

  return "UNKNOWN";
}

TemporalDiagnosticSnapshot TemporalDiagnosticsBuilder::Build(const TemporalDiagnosticsInput& input) const {
  TemporalDiagnosticSnapshot snapshot{
      .overall_status = TemporalHealthStatus::kOk,
      .imu_timing = input.imu_timing,
      .lidar_timing = input.lidar_timing,
      .has_imu_coverage = input.has_imu_coverage,
      .imu_coverage = input.imu_coverage,
      .lidar_scan_window = input.lidar_scan_window,
      .lidar_point_time = input.lidar_point_time,
      .imu_buffer_size = input.imu_buffer_size,
      .issues = {},
  };

  AddTimingIssue("IMU", input.imu_timing, &snapshot.issues);
  AddTimingIssue("LiDAR", input.lidar_timing, &snapshot.issues);

  if (!input.has_imu_coverage) {
    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kInfo,
            .title = "LiDAR scan has not been received yet",
            .explanation = "IMU coverage cannot be evaluated before the first LiDAR scan window is available.",
            .evidence = "imu_buffer_size=" + std::to_string(input.imu_buffer_size),
            .suggested_action = "Wait for LiDAR data or check the configured LiDAR topic.",
        },
        &snapshot);
  } else if (input.imu_coverage.health == causal_slam::coverage::ImuCoverageHealth::kDegraded) {
    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kDegraded,
            .title = "IMU does not properly cover the LiDAR scan window",
            .explanation = "Deskew and LiDAR-inertial fusion may be unreliable when IMU samples do not cover the scan interval.",
            .evidence = "reason=" + input.imu_coverage.reason +
                        ", imu_count_in_window=" + std::to_string(input.imu_coverage.imu_count_in_window) +
                        ", missing_prefix_ms=" + std::to_string(input.imu_coverage.missing_prefix_ms) +
                        ", missing_suffix_ms=" + std::to_string(input.imu_coverage.missing_suffix_ms) +
                        ", max_gap_inside_ms=" + std::to_string(input.imu_coverage.max_gap_inside_ms),
            .suggested_action = "Check IMU topic, timestamp base, sensor synchronization, and expected IMU period configuration.",
        },
        &snapshot);
  }

  if (input.lidar_point_time.has_time_candidate && !input.lidar_point_time.has_supported_time_field) {
    std::string action =
        "Use a supported point time representation, preferably FLOAT64 absolute seconds or UINT32 offset_time in nanoseconds.";

    if (input.lidar_point_time.inspection_reason == "absolute_float32_timestamp_precision_unsafe") {
      action = "Do not use FLOAT32 for absolute Unix-time-like timestamps. Use FLOAT64 timestamp or integer offset_time instead.";
    }

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .title = "LiDAR point timestamps were detected but not trusted",
            .explanation = "The cloud contains a time-like field, but the monitor rejected it as unsafe or unsupported.",
            .evidence = "field=" + input.lidar_point_time.field_name + ", datatype=" + input.lidar_point_time.field_datatype +
                        ", role=" + input.lidar_point_time.field_role + ", reason=" + input.lidar_point_time.inspection_reason,
            .suggested_action = action,
        },
        &snapshot);
  }

  if (input.lidar_point_time.extraction_attempted && !input.lidar_point_time.extraction_used) {
    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .title = "LiDAR point time extraction failed",
            .explanation = "A supported point time field was selected, but it did not produce a valid scan window.",
            .evidence = "reason=" + input.lidar_point_time.extraction_reason + ", unit=" + input.lidar_point_time.extraction_unit,
            .suggested_action = "Check PointCloud2 point_step, field offset, field datatype, and cloud data size.",
        },
        &snapshot);
  }

  if (input.lidar_scan_window.confidence == causal_slam::lidar::LidarScanWindowConfidence::kLow) {
    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .title = "LiDAR scan window has low confidence",
            .explanation = "The scan interval is estimated from fallback assumptions, not from point timestamps or measured scan metadata.",
            .evidence = "source=" + std::string(causal_slam::lidar::ToString(input.lidar_scan_window.source)) + ", reason=" +
                        input.lidar_scan_window.reason + ", duration_ms=" + std::to_string(input.lidar_scan_window.duration_ms),
            .suggested_action = "Prefer per-point timestamps, driver metadata, or validated measured header period.",
        },
        &snapshot);
  }

  for (const auto& issue : snapshot.issues) {
    snapshot.overall_status = MaxStatus(snapshot.overall_status, StatusFromSeverity(issue.severity));
  }

  return snapshot;
}

}  // namespace causal_slam::diagnostics