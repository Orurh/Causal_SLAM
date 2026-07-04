#include "temporal_diagnostics.h"

#include <string>
#include <utility>

namespace causal_slam::diagnostics {
namespace {

causal_slam::telemetry::TemporalHealthStatus MaxStatus(causal_slam::telemetry::TemporalHealthStatus lhs,
                                                       causal_slam::telemetry::TemporalHealthStatus rhs) {
  using causal_slam::telemetry::TemporalHealthStatus;

  if (lhs == TemporalHealthStatus::kInvalid || rhs == TemporalHealthStatus::kInvalid) {
    return TemporalHealthStatus::kInvalid;
  }

  if (lhs == TemporalHealthStatus::kDegraded || rhs == TemporalHealthStatus::kDegraded) {
    return TemporalHealthStatus::kDegraded;
  }

  if (lhs == TemporalHealthStatus::kWarning || rhs == TemporalHealthStatus::kWarning) {
    return TemporalHealthStatus::kWarning;
  }

  return TemporalHealthStatus::kOk;
}

causal_slam::telemetry::TemporalHealthStatus StatusFromSeverity(TemporalDiagnosticSeverity severity) {
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

TemporalDiagnosticSeverity SeverityFromTemporalHealth(causal_slam::telemetry::TemporalHealthStatus health) {
  switch (health) {
    case causal_slam::telemetry::TemporalHealthStatus::kOk:
      return TemporalDiagnosticSeverity::kInfo;
    case causal_slam::telemetry::TemporalHealthStatus::kWarning:
      return TemporalDiagnosticSeverity::kWarning;
    case causal_slam::telemetry::TemporalHealthStatus::kDegraded:
      return TemporalDiagnosticSeverity::kDegraded;
    case causal_slam::telemetry::TemporalHealthStatus::kInvalid:
      return TemporalDiagnosticSeverity::kInvalid;
  }

  return TemporalDiagnosticSeverity::kInvalid;
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

void AddTimingIssue(const std::string& stream_id, const std::string& stream_name, TemporalFaultReason reason,
                    const causal_slam::telemetry::TimingSummary& summary, std::vector<TemporalDiagnosticIssue>* issues) {
  if (summary.health == causal_slam::telemetry::TimingHealth::kOk) {
    return;
  }

  issues->push_back(TemporalDiagnosticIssue{
      .severity = SeverityFromTimingHealth(summary.health),
      .reason = reason,
      .title = stream_name + " stream timing is not stable",
      .explanation = "The message stream has temporal instability in the latest summary "
                     "window.",
      .evidence = "stream=" + stream_id + ", reason=" + summary.reason + ", window_count=" + std::to_string(summary.window_count) +
                  ", last_period_ms=" + std::to_string(summary.last_period_ms) + ", window_max_jitter_ms=" +
                  std::to_string(summary.window_max_jitter_ms) + ", window_gap_count=" + std::to_string(summary.window_gap_count) +
                  ", window_reordered_count=" + std::to_string(summary.window_reordered_count),
      .suggested_action = "Check sensor driver timing, QoS, CPU load, transport latency, and "
                          "timestamp source.",
  });
}

TemporalFaultReason FaultReasonFromTransformStatus(causal_slam::transform::TransformLookupStatus status) {
  switch (status) {
    case causal_slam::transform::TransformLookupStatus::kOk:
      return TemporalFaultReason::kNone;
    case causal_slam::transform::TransformLookupStatus::kLookupFailed:
      return TemporalFaultReason::kTfLookupFailed;
    case causal_slam::transform::TransformLookupStatus::kExtrapolationRequired:
      return TemporalFaultReason::kTfExtrapolationRequired;
    case causal_slam::transform::TransformLookupStatus::kTransformAgeTooHigh:
      return TemporalFaultReason::kTfAgeTooHigh;
    case causal_slam::transform::TransformLookupStatus::kTransformFromFuture:
      return TemporalFaultReason::kTfTransformFromFuture;
  }

  return TemporalFaultReason::kNone;
}

std::string TitleFromTransformStatus(causal_slam::transform::TransformLookupStatus status) {
  switch (status) {
    case causal_slam::transform::TransformLookupStatus::kOk:
      return "TF transform is temporally valid";
    case causal_slam::transform::TransformLookupStatus::kLookupFailed:
      return "TF lookup failed for the sensor timestamp";
    case causal_slam::transform::TransformLookupStatus::kExtrapolationRequired:
      return "TF lookup required extrapolation";
    case causal_slam::transform::TransformLookupStatus::kTransformAgeTooHigh:
      return "TF transform age is too high";
    case causal_slam::transform::TransformLookupStatus::kTransformFromFuture:
      return "TF transform timestamp is in the future";
  }

  return "TF transform is not temporally valid";
}

std::string SuggestedActionFromTransformStatus(causal_slam::transform::TransformLookupStatus status) {
  switch (status) {
    case causal_slam::transform::TransformLookupStatus::kOk:
      return "No action required.";
    case causal_slam::transform::TransformLookupStatus::kLookupFailed:
      return "Check TF publishers, frame names, static transforms, and whether "
             "the requested timestamp is available in the TF buffer.";
    case causal_slam::transform::TransformLookupStatus::kExtrapolationRequired:
      return "Check sensor timestamp source, TF publication latency, and clock "
             "synchronization. Avoid using transforms outside the known TF "
             "buffer range.";
    case causal_slam::transform::TransformLookupStatus::kTransformAgeTooHigh:
      return "Check transform publisher rate, odometry latency, TF buffer age, "
             "and whether the SLAM pipeline is using stale transforms.";
    case causal_slam::transform::TransformLookupStatus::kTransformFromFuture:
      return "Check clock domains, timestamp source, rosbag playback timing, "
             "and whether transforms are being stamped ahead of sensor data.";
  }

  return "Check TF timing and frame configuration.";
}

void AddIssueAndUpdateStatus(TemporalDiagnosticIssue issue, TemporalDiagnosticSnapshot* snapshot) {
  snapshot->overall_status = MaxStatus(snapshot->overall_status, StatusFromSeverity(issue.severity));
  snapshot->issues.push_back(std::move(issue));
}

void AddTransformIssue(const causal_slam::transform::TransformAgeSummary& summary, TemporalDiagnosticSnapshot* snapshot) {
  if (summary.status == causal_slam::transform::TransformLookupStatus::kOk) {
    return;
  }

  const auto reason = FaultReasonFromTransformStatus(summary.status);
  if (reason == TemporalFaultReason::kNone) {
    return;
  }

  AddIssueAndUpdateStatus(
      TemporalDiagnosticIssue{
          .severity = SeverityFromTemporalHealth(summary.health),
          .reason = reason,
          .title = TitleFromTransformStatus(summary.status),
          .explanation = "The transform lookup result is not temporally safe for the "
                         "sensor measurement timestamp.",
          .evidence = "target_frame=" + summary.target_frame + ", source_frame=" + summary.source_frame +
                      ", status=" + std::string(causal_slam::transform::ToString(summary.status)) + ", transform_age_ms=" +
                      std::to_string(summary.transform_age_ms) + ", receive_delay_ms=" + std::to_string(summary.receive_delay_ms) +
                      ", adapter_detail=" + summary.adapter_detail + ", reason=" + summary.reason,
          .suggested_action = SuggestedActionFromTransformStatus(summary.status),
      },
      snapshot);
}

TemporalFaultReason StreamTimingFaultReason(causal_slam::telemetry::TemporalStreamId stream_id) {
  switch (stream_id) {
    case causal_slam::telemetry::TemporalStreamId::kImu:
      return TemporalFaultReason::kImuStreamTimingUnstable;

    case causal_slam::telemetry::TemporalStreamId::kLidar:
      return TemporalFaultReason::kLidarStreamTimingUnstable;

    case causal_slam::telemetry::TemporalStreamId::kUnknown:
    case causal_slam::telemetry::TemporalStreamId::kCamera:
    case causal_slam::telemetry::TemporalStreamId::kRadar:
    case causal_slam::telemetry::TemporalStreamId::kGnss:
    case causal_slam::telemetry::TemporalStreamId::kTf:
      return TemporalFaultReason::kStreamTimingUnstable;
  }

  return TemporalFaultReason::kStreamTimingUnstable;
}

std::string StreamDisplayName(causal_slam::telemetry::TemporalStreamId stream_id) {
  switch (stream_id) {
    case causal_slam::telemetry::TemporalStreamId::kImu:
      return "IMU";

    case causal_slam::telemetry::TemporalStreamId::kLidar:
      return "LiDAR";

    case causal_slam::telemetry::TemporalStreamId::kCamera:
      return "Camera";

    case causal_slam::telemetry::TemporalStreamId::kRadar:
      return "Radar";

    case causal_slam::telemetry::TemporalStreamId::kGnss:
      return "GNSS";

    case causal_slam::telemetry::TemporalStreamId::kTf:
      return "TF";

    case causal_slam::telemetry::TemporalStreamId::kUnknown:
      return "Unknown";
  }

  return std::string{causal_slam::telemetry::ToString(stream_id)};
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
    case TemporalFaultReason::kImuStreamTimingUnstable:
      return "imu_stream_timing_unstable";
    case TemporalFaultReason::kLidarStreamTimingUnstable:
      return "lidar_stream_timing_unstable";
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
    case TemporalFaultReason::kTfLookupFailed:
      return "tf_lookup_failed";
    case TemporalFaultReason::kTfExtrapolationRequired:
      return "tf_extrapolation_required";
    case TemporalFaultReason::kTfAgeTooHigh:
      return "tf_age_too_high";
    case TemporalFaultReason::kTfTransformFromFuture:
      return "tf_transform_from_future";
    case TemporalFaultReason::kNoImuSampleReceivedYet:
      return "no_imu_sample_received_yet";
    case TemporalFaultReason::kLidarStreamStale:
      return "lidar_stream_stale";
    case TemporalFaultReason::kImuStreamStale:
      return "imu_stream_stale";
  }

  return "unknown";
}

TemporalDiagnosticSnapshot TemporalDiagnosticsBuilder::Build(const causal_slam::model::TemporalObservation& observation) const {
  TemporalDiagnosticSnapshot snapshot{
      .overall_status = causal_slam::telemetry::TemporalHealthStatus::kOk,
      .observation = observation,
      .issues = {},
  };

  for (const auto& stream : observation.streams) {
    AddTimingIssue(std::string{causal_slam::telemetry::ToString(stream.id)}, StreamDisplayName(stream.id),
                   StreamTimingFaultReason(stream.id), stream.timing, &snapshot.issues);
  }

  if (!observation.imu_coverage.has_value()) {
    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kInvalid,
            .reason = TemporalFaultReason::kNoLidarScanReceivedYet,
            .title = "LiDAR scan has not been received yet",
            .explanation = "IMU coverage cannot be evaluated before the first LiDAR scan "
                           "window is available.",
            .evidence = "imu_buffer_size=" + std::to_string(observation.imu_buffer_size),
            .suggested_action = "Wait for LiDAR data or check the configured LiDAR topic.",
        },
        &snapshot);
  } else if (observation.imu_coverage->health == causal_slam::coverage::ImuCoverageHealth::kDegraded) {
    const auto& imu_coverage = *observation.imu_coverage;

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kDegraded,
            .reason = TemporalFaultReason::kImuWindowIncomplete,
            .title = "IMU does not properly cover the LiDAR scan window",
            .explanation = "Deskew and LiDAR-inertial fusion may be unreliable when IMU "
                           "samples do not cover the scan interval.",
            .evidence = "reason=" + imu_coverage.reason + ", imu_count_in_window=" + std::to_string(imu_coverage.imu_count_in_window) +
                        ", missing_prefix_ms=" + std::to_string(imu_coverage.missing_prefix_ms) +
                        ", missing_suffix_ms=" + std::to_string(imu_coverage.missing_suffix_ms) +
                        ", max_gap_inside_ms=" + std::to_string(imu_coverage.max_gap_inside_ms),
            .suggested_action = "Check IMU topic, timestamp base, sensor synchronization, and "
                                "expected IMU period configuration.",
        },
        &snapshot);
  }

  if (observation.lidar_point_time.has_value() && observation.lidar_point_time->has_time_candidate &&
      !observation.lidar_point_time->has_supported_time_field) {
    const auto& point_time = *observation.lidar_point_time;

    std::string action =
        "Use a supported point time representation, preferably FLOAT64 "
        "absolute seconds or UINT32 offset_time in nanoseconds.";

    if (point_time.inspection_reason == "absolute_float32_timestamp_precision_unsafe") {
      action =
          "Do not use FLOAT32 for absolute Unix-time-like timestamps. Use "
          "FLOAT64 timestamp or integer offset_time instead.";
    }

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .reason = TemporalFaultReason::kLidarPointTimeUnsupported,
            .title = "LiDAR point timestamps were detected but not trusted",
            .explanation = "The cloud contains a time-like field, but the monitor "
                           "rejected it as unsafe or unsupported.",
            .evidence = "field=" + point_time.field_name + ", datatype=" + point_time.field_datatype + ", role=" + point_time.field_role +
                        ", reason=" + point_time.inspection_reason,
            .suggested_action = action,
        },
        &snapshot);
  }

  if (observation.lidar_point_time.has_value() && observation.lidar_point_time->extraction_attempted &&
      !observation.lidar_point_time->extraction_used) {
    const auto& point_time = *observation.lidar_point_time;

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .reason = TemporalFaultReason::kLidarPointTimeExtractionFailed,
            .title = "LiDAR point time extraction failed",
            .explanation = "A supported point time field was selected, but it did not "
                           "produce a valid scan window.",
            .evidence = "reason=" + point_time.extraction_reason + ", unit=" + point_time.extraction_unit,
            .suggested_action = "Check PointCloud2 point_step, field offset, field datatype, "
                                "and cloud data size.",
        },
        &snapshot);
  }

  if (observation.lidar_scan_window.has_value() &&
      observation.lidar_scan_window->confidence == causal_slam::lidar::LidarScanWindowConfidence::kLow) {
    const auto& scan_window = *observation.lidar_scan_window;

    AddIssueAndUpdateStatus(
        TemporalDiagnosticIssue{
            .severity = TemporalDiagnosticSeverity::kWarning,
            .reason = TemporalFaultReason::kLidarScanWindowLowConfidence,
            .title = "LiDAR scan window has low confidence",
            .explanation = "The scan interval is estimated from fallback assumptions, "
                           "not from point timestamps or measured scan metadata.",
            .evidence = "source=" + std::string(causal_slam::lidar::ToString(scan_window.source)) + ", reason=" + scan_window.reason +
                        ", duration_ms=" + std::to_string(scan_window.duration_ms),
            .suggested_action = "Prefer per-point timestamps, driver metadata, or validated "
                                "measured header period.",
        },
        &snapshot);
  }

  for (const auto& transform_age : observation.transform_ages) {
    AddTransformIssue(transform_age, &snapshot);
  }

  for (const auto& issue : snapshot.issues) {
    snapshot.overall_status = MaxStatus(snapshot.overall_status, StatusFromSeverity(issue.severity));
  }

  return snapshot;
}

}  // namespace causal_slam::diagnostics
