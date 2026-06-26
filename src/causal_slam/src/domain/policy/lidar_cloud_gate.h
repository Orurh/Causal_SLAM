#pragma once

#include <cstdint>
#include <string_view>

#include "domain/telemetry/temporal_health.h"

namespace causal_slam::policy {

enum class LidarGateMode : std::uint8_t {
  kObserve,
  kDropInvalid,
  kDropDegraded,
  kStrict,
};

enum class LidarCloudGateReason : std::uint8_t {
  kObserveMode,
  kInsufficientTimingEvidence,
  kTemporalHealthOk,
  kTemporalHealthWarning,
  kTemporalHealthDegraded,
  kTemporalHealthInvalid,
};

struct LidarCloudGateConfig {
  LidarGateMode mode{LidarGateMode::kObserve};

  std::uint64_t min_total_imu_samples_before_forward{0};
  std::uint64_t min_window_imu_samples_before_forward{0};
};

struct LidarCloudGateInput {
  causal_slam::telemetry::TemporalHealthStatus health{causal_slam::telemetry::TemporalHealthStatus::kOk};
  std::uint64_t total_imu_samples{0};
  std::uint64_t window_imu_samples{0};
};

struct LidarCloudGateResult {
  bool should_forward{true};
  LidarCloudGateReason reason{LidarCloudGateReason::kObserveMode};
};

[[nodiscard]] const char* ToString(LidarGateMode mode);
[[nodiscard]] const char* ToString(LidarCloudGateReason reason);

[[nodiscard]] LidarGateMode ParseLidarGateMode(std::string_view value);

[[nodiscard]] bool IsActiveLidarGateMode(LidarGateMode mode);

[[nodiscard]] bool HasMinimumTimingEvidenceForActiveGate(const LidarCloudGateConfig& config, const LidarCloudGateInput& input);

[[nodiscard]] bool ShouldForwardLidarCloud(LidarGateMode mode, causal_slam::telemetry::TemporalHealthStatus health);

[[nodiscard]] LidarCloudGateResult EvaluateLidarCloudGate(const LidarCloudGateConfig& config, const LidarCloudGateInput& input);

}  // namespace causal_slam::policy