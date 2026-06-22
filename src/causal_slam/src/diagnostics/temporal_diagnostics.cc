#include "diagnostics/temporal_diagnostics.h"

#include "policy/map_update_decision.h"

#include <string>
#include <utility>

namespace causal_slam::diagnostics {
namespace {

causal_slam::telemetry::TemporalHealthStatus MaxStatus(
    causal_slam::telemetry::TemporalHealthStatus lhs,
    causal_slam::telemetry::TemporalHealthStatus rhs) {
  using causal_slam::telemetry::TemporalHealthStatus;

  if (lhs == TemporalHealthStatus::kInvalid ||
      rhs == TemporalHealthStatus::kInvalid) {
    return TemporalHealthStatus::kInvalid;
  }

  if (lhs == TemporalHealthStatus::kDegraded ||
      rhs == TemporalHealthStatus::kDegraded) {
    return TemporalHealthStatus::kDegraded;
  }

  if (lhs == TemporalHealthStatus::kWarning ||
      rhs == TemporalHealthStatus::kWarning) {
    return TemporalHealthStatus::kWarning;
  }

  return TemporalHealthStatus::kOk;
}

causal_slam::telemetry::TemporalHealthStatus StatusFromSeverity(
    TemporalDiagnosticSeverity severity) {
  using causal_slam::telemetry::TemporalHealthStatus;

  switch (severity) {
    case TemporalDiagnosticSeverity::kInfo:
      return TemporalHealthStatus::kOk;
    case TemporalDiagnosticSeverity::kWarning:
      return TemporalHealthStatus::kWarning;
    case TemporalDiagnosticSeverity::kDegraded:
      return TemporalHealthStatus::kDegraded;
    case TemporalDiagnosticSeverity::kInvalid:
      return TemporalHealthStatus::kInvalid;
  }

  return TemporalHealthStatus::kInvalid;
}

TemporalDiagnosticSeverity SeverityFromTimingHealth(
    causal_slam::telemetry::TimingHealth health) {
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

void AddTimingIssue(
    const char* stream_name,
    const causal_slam::telemetry::TimingSummary& summary,
    std::vector<TemporalDiagnosticIssue>* issues) {
  if (summary.health == causal_slam::telemetry::TimingHealth::kOk) {
    return;
  }

  issues->push_back(TemporalDiagnosticIssue{
      .severity = SeverityFromTimingHealth(summary.health),
      .reason = TemporalFaultReason::kStreamTimingUnstable,
      .title = std::string(stream_name) + " timing is not stable",
      .explanation =
          "The message stream has temporal instability in the latest summary "
          "window.",
      .evidence =
          "reason=" + summary.reason +
          ", window_count=" + std::to_string(summary.window_count) +
          ", last_period_ms=" + std::to_string(summary.last_period_ms) +
          ", window_max_jitter_ms=" +
          std::to_string(summary.window_max_jitter_ms) +
          ", window_gap_count=" + std::to_string(summary.window_gap_count) +
          ", window_reordered_count=" +
          std::to_string(summary.window_reordered_count),
      .suggested_action =
          "Check sensor driver timing, QoS, CPU load, transport latency, and "
          "timestamp source.",
  });
}

void AddIssueAndUpdateStatus(TemporalDiagnosticIssue issue,
                             TemporalDiagnosticSnapshot* snapshot) {
  snapshot->overall_status = MaxStatus(
      snapshot->overall_status, StatusFromSeverity(issue.severity));
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
    case TemporalDiagnosticSeverity::kInvalid:
      return "INVALID";
  }

  return "UNKNOWN";
}

const char* ToString(TemporalFaultReason reason) {
  switch (reason) {
    case TemporalFaultReason::kNone:
      return "none";
    case TemporalFaultReason::kStreamTimingUnstable:
      return "stream_timing_unstable";
    case TemporalFaultReason::kNoLidarScanReceivedYet:
      return "no_lidar_scan_received_yet";
    case TemporalFaultReason::kImuWindowIncomplete:
      return "imu_window_incomplete";
    case TemporalFaultReason::kLidarPointTimeUnsupported:
      return "lidar_point_time_unsupported";
    case TemporalFaultReason::kLidarPointTimeExtractionFailed:
      return "lidar_point_time_extraction_failed";
    case TemporalFaultReason::kLidarScanWindowLowConfidence:
      return "lidar_scan_window_low_confidence";
  }

  return "unknown";
}

TemporalDiagnosticSnapshot TemporalDiagnosticsBuilder::Build(
    const causal_slam::model::TemporalObservation& observation) const {
  TemporalDiagnosticSnapshot snapshot{
      .overall_status = causal_slam::telemetry::TemporalHealthStatus::kOk,
      .map_update_decision =
          causal_slam::policy::DecideMapUpdate(
              causal_slam::telemetry::TemporalHealthStatus::kOk),
      .observation = observation,
      .issues = {},
  };

  for (const auto& stream : observation.streams) {
    AddTimingIssue(causal_slam::telemetry::ToString(stream.id),
                   stream.timing,
                   &snapshot.issues);
  }

  if (!observation.imu_coverage.has_value()) {
    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kInfo,
            .reason = TemporalFaultReason::kNoLidarScanReceivedYet,
            .title = "LiDAR scan has not been received yet",
            .explanation =
                "IMU coverage cannot be evaluated before the first LiDAR scan "
                "window is available.",
            .evidence =
                "imu_buffer_size=" + std::to_string(observation.imu_buffer_size),
            .suggested_action =
                "Wait for LiDAR data or check the configured LiDAR topic.",
        },
        &snapshot);
  } else if (observation.imu_coverage->health ==
             causal_slam::coverage::ImuCoverageHealth::kDegraded) {
    const auto& imu_coverage = *observation.imu_coverage;

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kDegraded,
            .reason = TemporalFaultReason::kImuWindowIncomplete,
            .title = "IMU does not properly cover the LiDAR scan window",
            .explanation =
                "Deskew and LiDAR-inertial fusion may be unreliable when IMU "
                "samples do not cover the scan interval.",
            .evidence =
                "reason=" + imu_coverage.reason +
                ", imu_count_in_window=" +
                std::to_string(imu_coverage.imu_count_in_window) +
                ", missing_prefix_ms=" +
                std::to_string(imu_coverage.missing_prefix_ms) +
                ", missing_suffix_ms=" +
                std::to_string(imu_coverage.missing_suffix_ms) +
                ", max_gap_inside_ms=" +
                std::to_string(imu_coverage.max_gap_inside_ms),
            .suggested_action =
                "Check IMU topic, timestamp base, sensor synchronization, and "
                "expected IMU period configuration.",
        },
        &snapshot);
  }

  if (observation.lidar_point_time.has_value() &&
      observation.lidar_point_time->has_time_candidate &&
      !observation.lidar_point_time->has_supported_time_field) {
    const auto& point_time = *observation.lidar_point_time;

    std::string action =
        "Use a supported point time representation, preferably FLOAT64 "
        "absolute seconds or UINT32 offset_time in nanoseconds.";

    if (point_time.inspection_reason ==
        "absolute_float32_timestamp_precision_unsafe") {
      action =
          "Do not use FLOAT32 for absolute Unix-time-like timestamps. Use "
          "FLOAT64 timestamp or integer offset_time instead.";
    }

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .reason = TemporalFaultReason::kLidarPointTimeUnsupported,
            .title = "LiDAR point timestamps were detected but not trusted",
            .explanation =
                "The cloud contains a time-like field, but the monitor "
                "rejected it as unsafe or unsupported.",
            .evidence =
                "field=" + point_time.field_name +
                ", datatype=" + point_time.field_datatype +
                ", role=" + point_time.field_role +
                ", reason=" + point_time.inspection_reason,
            .suggested_action = action,
        },
        &snapshot);
  }

  if (observation.lidar_point_time.has_value() &&
      observation.lidar_point_time->extraction_attempted &&
      !observation.lidar_point_time->extraction_used) {
    const auto& point_time = *observation.lidar_point_time;

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .reason = TemporalFaultReason::kLidarPointTimeExtractionFailed,
            .title = "LiDAR point time extraction failed",
            .explanation =
                "A supported point time field was selected, but it did not "
                "produce a valid scan window.",
            .evidence =
                "reason=" + point_time.extraction_reason +
                ", unit=" + point_time.extraction_unit,
            .suggested_action =
                "Check PointCloud2 point_step, field offset, field datatype, "
                "and cloud data size.",
        },
        &snapshot);
  }

  if (observation.lidar_scan_window.has_value() &&
      observation.lidar_scan_window->confidence ==
          causal_slam::lidar::LidarScanWindowConfidence::kLow) {
    const auto& scan_window = *observation.lidar_scan_window;

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .reason = TemporalFaultReason::kLidarScanWindowLowConfidence,
            .title = "LiDAR scan window has low confidence",
            .explanation =
                "The scan interval is estimated from fallback assumptions, "
                "not from point timestamps or measured scan metadata.",
            .evidence =
                "source=" +
                std::string(causal_slam::lidar::ToString(scan_window.source)) +
                ", reason=" + scan_window.reason +
                ", duration_ms=" + std::to_string(scan_window.duration_ms),
            .suggested_action =
                "Prefer per-point timestamps, driver metadata, or validated "
                "measured header period.",
        },
        &snapshot);
  }

  for (const auto& issue : snapshot.issues) {
    snapshot.overall_status = MaxStatus(
        snapshot.overall_status, StatusFromSeverity(issue.severity));
  }

  snapshot.map_update_decision =
      causal_slam::policy::DecideMapUpdate(snapshot.overall_status);

  return snapshot;
}

}  // namespace causal_slam::diagnostics
