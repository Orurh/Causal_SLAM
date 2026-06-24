#pragma once

#include <cstdint>
#include <string>

#include "telemetry/temporal_health.h"
#include "transform/transform_lookup_observation.h"

namespace causal_slam::transform {

enum class TransformLookupStatus {
  kOk,
  kLookupFailed,
  kExtrapolationRequired,
  kTransformAgeTooHigh,
  kTransformFromFuture,
};

[[nodiscard]] const char* ToString(TransformLookupStatus status);

struct TransformAgeAnalyzerConfig {
  double max_transform_age_ms{50.0};
  double max_future_tolerance_ms{1.0};

  causal_slam::telemetry::TemporalHealthStatus lookup_failed_health{
      causal_slam::telemetry::TemporalHealthStatus::kInvalid};
  causal_slam::telemetry::TemporalHealthStatus extrapolation_health{
      causal_slam::telemetry::TemporalHealthStatus::kDegraded};
  causal_slam::telemetry::TemporalHealthStatus stale_transform_health{
      causal_slam::telemetry::TemporalHealthStatus::kDegraded};
  causal_slam::telemetry::TemporalHealthStatus future_transform_health{
      causal_slam::telemetry::TemporalHealthStatus::kDegraded};
};

struct TransformAgeSummary {
  causal_slam::telemetry::TemporalHealthStatus health{
      causal_slam::telemetry::TemporalHealthStatus::kOk};

  TransformLookupStatus status{TransformLookupStatus::kOk};

  std::string target_frame;
  std::string source_frame;

  double transform_age_ms{0.0};
  double receive_delay_ms{0.0};

  std::string reason{"ok"};
  std::string adapter_detail;
};

class TransformAgeAnalyzer final {
 public:
  explicit TransformAgeAnalyzer(
      TransformAgeAnalyzerConfig config = TransformAgeAnalyzerConfig{});

  [[nodiscard]] TransformAgeSummary Analyze(
      const TransformLookupObservation& observation) const;

 private:
  TransformAgeAnalyzerConfig config_;
};

}  // namespace causal_slam::transform
