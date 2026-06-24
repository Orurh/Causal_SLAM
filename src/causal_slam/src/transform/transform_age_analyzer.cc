#include "transform/transform_age_analyzer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace causal_slam::transform {
namespace {

constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

double NanosecondsToMilliseconds(std::int64_t value_ns) {
  return static_cast<double>(value_ns) / kNanosecondsPerMillisecond;
}

}  // namespace

const char* ToString(TransformLookupStatus status) {
  switch (status) {
    case TransformLookupStatus::kOk:
      return "ok";
    case TransformLookupStatus::kLookupFailed:
      return "tf_lookup_failed";
    case TransformLookupStatus::kExtrapolationRequired:
      return "tf_extrapolation_required";
    case TransformLookupStatus::kTransformAgeTooHigh:
      return "tf_age_too_high";
    case TransformLookupStatus::kTransformFromFuture:
      return "tf_transform_from_future";
  }

  return "unknown";
}

TransformAgeAnalyzer::TransformAgeAnalyzer(TransformAgeAnalyzerConfig config)
    : config_(config) {
  config_.max_transform_age_ms =
      std::max(config_.max_transform_age_ms, 0.0);
  config_.max_future_tolerance_ms =
      std::max(config_.max_future_tolerance_ms, 0.0);
}

TransformAgeSummary TransformAgeAnalyzer::Analyze(
    const TransformLookupObservation& observation) const {
  TransformAgeSummary summary;
  summary.target_frame = observation.target_frame;
  summary.source_frame = observation.source_frame;
  summary.adapter_detail = observation.failure_reason;

  summary.transform_age_ms = NanosecondsToMilliseconds(
      observation.requested_stamp_ns - observation.transform_stamp_ns);
  summary.receive_delay_ms = NanosecondsToMilliseconds(
      observation.receive_time_ns - observation.requested_stamp_ns);

  if (!observation.lookup_success) {
    summary.health = config_.lookup_failed_health;
    summary.status = TransformLookupStatus::kLookupFailed;
    summary.reason = ToString(summary.status);
    return summary;
  }

  if (observation.extrapolation_required) {
    summary.health = config_.extrapolation_health;
    summary.status = TransformLookupStatus::kExtrapolationRequired;
    summary.reason = ToString(summary.status);
    return summary;
  }

  if (summary.transform_age_ms < -config_.max_future_tolerance_ms) {
    summary.health = config_.future_transform_health;
    summary.status = TransformLookupStatus::kTransformFromFuture;
    summary.reason = ToString(summary.status);
    return summary;
  }

  if (std::abs(summary.transform_age_ms) > config_.max_transform_age_ms) {
    summary.health = config_.stale_transform_health;
    summary.status = TransformLookupStatus::kTransformAgeTooHigh;
    summary.reason = ToString(summary.status);
    return summary;
  }

  summary.health = causal_slam::telemetry::TemporalHealthStatus::kOk;
  summary.status = TransformLookupStatus::kOk;
  summary.reason = ToString(summary.status);
  return summary;
}

}  // namespace causal_slam::transform
