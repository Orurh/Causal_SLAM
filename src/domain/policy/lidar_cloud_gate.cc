#include "domain/policy/lidar_cloud_gate.h"

namespace causal_slam::policy {
namespace {

[[nodiscard]] LidarCloudGateReason ReasonForHealth(causal_slam::telemetry::TemporalHealthStatus health) {
  using causal_slam::telemetry::TemporalHealthStatus;

  switch (health) {
    case TemporalHealthStatus::kOk:
      return LidarCloudGateReason::kTemporalHealthOk;
    case TemporalHealthStatus::kWarning:
      return LidarCloudGateReason::kTemporalHealthWarning;
    case TemporalHealthStatus::kDegraded:
      return LidarCloudGateReason::kTemporalHealthDegraded;
    case TemporalHealthStatus::kInvalid:
      return LidarCloudGateReason::kTemporalHealthInvalid;
  }

  return LidarCloudGateReason::kTemporalHealthInvalid;
}

}  // namespace

const char* ToString(LidarGateMode mode) {
  switch (mode) {
    case LidarGateMode::kObserve:
      return "observe";
    case LidarGateMode::kDropInvalid:
      return "drop_invalid";
    case LidarGateMode::kDropDegraded:
      return "drop_degraded";
    case LidarGateMode::kStrict:
      return "strict";
  }

  return "observe";
}

const char* ToString(LidarCloudGateReason reason) {
  switch (reason) {
    case LidarCloudGateReason::kObserveMode:
      return "observe_mode";
    case LidarCloudGateReason::kInsufficientTimingEvidence:
      return "insufficient_timing_evidence";
    case LidarCloudGateReason::kTemporalHealthOk:
      return "temporal_health_ok";
    case LidarCloudGateReason::kTemporalHealthWarning:
      return "temporal_health_warning";
    case LidarCloudGateReason::kTemporalHealthDegraded:
      return "temporal_health_degraded";
    case LidarCloudGateReason::kTemporalHealthInvalid:
      return "temporal_health_invalid";
  }

  return "temporal_health_invalid";
}

LidarGateMode ParseLidarGateMode(std::string_view value) {
  if (value == "observe") {
    return LidarGateMode::kObserve;
  }

  if (value == "drop_invalid") {
    return LidarGateMode::kDropInvalid;
  }

  if (value == "drop_degraded") {
    return LidarGateMode::kDropDegraded;
  }

  if (value == "strict") {
    return LidarGateMode::kStrict;
  }

  return LidarGateMode::kObserve;
}

bool IsActiveLidarGateMode(LidarGateMode mode) {
  return mode != LidarGateMode::kObserve;
}

bool IsHardFusionBlockingReason(std::string_view reason) {
  return reason == "no_imu_sample_received_yet" || reason == "imu_window_incomplete" || reason == "imu_window_empty" ||
         reason == "imu_window_missing_prefix" || reason == "imu_window_missing_suffix" || reason == "imu_window_internal_gap" ||
         reason == "invalid_scan_window" || reason == "timestamp_invalid" || reason == "message_reordering_detected" ||
         reason == "lidar_point_time_unsupported" || reason == "lidar_point_time_extraction_failed" || reason == "imu_stream_stale" ||
         reason == "lidar_stream_stale";
}

bool HasMinimumTimingEvidenceForActiveGate(const LidarCloudGateConfig& config, const LidarCloudGateInput& input) {
  return input.total_imu_samples >= config.min_total_imu_samples_before_forward &&
         input.window_imu_samples >= config.min_window_imu_samples_before_forward;
}

bool ShouldForwardLidarCloud(LidarGateMode mode, causal_slam::telemetry::TemporalHealthStatus health, bool has_hard_fusion_blocker) {
  using causal_slam::telemetry::TemporalHealthStatus;

  switch (mode) {
    case LidarGateMode::kObserve:
      return true;

    case LidarGateMode::kDropInvalid:
      return health != TemporalHealthStatus::kInvalid;

    case LidarGateMode::kDropDegraded:
      // Stream-level jitter is diagnostic-only for downstream LIO.
      // Drop only invalid state or scan-level hard fusion blockers.
      return health != TemporalHealthStatus::kInvalid && !has_hard_fusion_blocker;

    case LidarGateMode::kStrict:
      return health == TemporalHealthStatus::kOk;
  }

  return true;
}

LidarCloudGateResult EvaluateLidarCloudGate(const LidarCloudGateConfig& config, const LidarCloudGateInput& input) {
  if (!IsActiveLidarGateMode(config.mode)) {
    return LidarCloudGateResult{
        .should_forward = true,
        .reason = LidarCloudGateReason::kObserveMode,
    };
  }

  if (!HasMinimumTimingEvidenceForActiveGate(config, input)) {
    return LidarCloudGateResult{
        .should_forward = false,
        .reason = LidarCloudGateReason::kInsufficientTimingEvidence,
    };
  }

  return LidarCloudGateResult{
      .should_forward = ShouldForwardLidarCloud(config.mode, input.health, input.has_hard_fusion_blocker),
      .reason = ReasonForHealth(input.health),
  };
}

}  // namespace causal_slam::policy