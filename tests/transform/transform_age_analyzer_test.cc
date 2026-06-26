#include "domain/sensors/transform/transform_age_analyzer.h"

#include <cstdint>

#include <gtest/gtest.h>

namespace causal_slam::transform {
namespace {

constexpr std::int64_t kNanosecondsPerMillisecond = 1'000'000LL;

std::int64_t Ms(std::int64_t value) {
  return value * kNanosecondsPerMillisecond;
}

TransformLookupObservation MakeBaseObservation() {
  TransformLookupObservation observation;
  observation.target_frame = "odom";
  observation.source_frame = "base_link";
  observation.requested_stamp_ns = Ms(1000);
  observation.transform_stamp_ns = Ms(990);
  observation.receive_time_ns = Ms(1010);
  observation.lookup_success = true;
  observation.extrapolation_required = false;
  observation.failure_reason = "";
  return observation;
}

TEST(TransformAgeAnalyzerTest, FreshTransformIsOk) {
  TransformAgeAnalyzerConfig config;
  config.max_transform_age_ms = 50.0;
  config.max_future_tolerance_ms = 1.0;

  const TransformAgeAnalyzer analyzer{config};
  const auto summary = analyzer.Analyze(MakeBaseObservation());

  EXPECT_EQ(summary.health,
            causal_slam::telemetry::TemporalHealthStatus::kOk);
  EXPECT_EQ(summary.status, TransformLookupStatus::kOk);
  EXPECT_EQ(summary.reason, "ok");
  EXPECT_DOUBLE_EQ(summary.transform_age_ms, 10.0);
  EXPECT_DOUBLE_EQ(summary.receive_delay_ms, 10.0);
}

TEST(TransformAgeAnalyzerTest, FailedLookupIsInvalidByDefault) {
  TransformAgeAnalyzer analyzer;

  auto observation = MakeBaseObservation();
  observation.lookup_success = false;
  observation.failure_reason = "frame_not_found";

  const auto summary = analyzer.Analyze(observation);

  EXPECT_EQ(summary.health,
            causal_slam::telemetry::TemporalHealthStatus::kInvalid);
  EXPECT_EQ(summary.status, TransformLookupStatus::kLookupFailed);
  EXPECT_EQ(summary.reason, "tf_lookup_failed");
  EXPECT_EQ(summary.adapter_detail, "frame_not_found");
}

TEST(TransformAgeAnalyzerTest, ExtrapolationIsDegradedByDefault) {
  TransformAgeAnalyzer analyzer;

  auto observation = MakeBaseObservation();
  observation.extrapolation_required = true;
  observation.failure_reason = "extrapolation_into_future";

  const auto summary = analyzer.Analyze(observation);

  EXPECT_EQ(summary.health,
            causal_slam::telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_EQ(summary.status, TransformLookupStatus::kExtrapolationRequired);
  EXPECT_EQ(summary.reason, "tf_extrapolation_required");
  EXPECT_EQ(summary.adapter_detail, "extrapolation_into_future");
}

TEST(TransformAgeAnalyzerTest, StaleTransformIsDegradedByDefault) {
  TransformAgeAnalyzerConfig config;
  config.max_transform_age_ms = 50.0;

  const TransformAgeAnalyzer analyzer{config};

  auto observation = MakeBaseObservation();
  observation.transform_stamp_ns = Ms(900);

  const auto summary = analyzer.Analyze(observation);

  EXPECT_EQ(summary.health,
            causal_slam::telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_EQ(summary.status, TransformLookupStatus::kTransformAgeTooHigh);
  EXPECT_EQ(summary.reason, "tf_age_too_high");
  EXPECT_DOUBLE_EQ(summary.transform_age_ms, 100.0);
}

TEST(TransformAgeAnalyzerTest, FutureTransformBeyondToleranceIsDegraded) {
  TransformAgeAnalyzerConfig config;
  config.max_transform_age_ms = 50.0;
  config.max_future_tolerance_ms = 2.0;

  const TransformAgeAnalyzer analyzer{config};

  auto observation = MakeBaseObservation();
  observation.transform_stamp_ns = Ms(1005);

  const auto summary = analyzer.Analyze(observation);

  EXPECT_EQ(summary.health,
            causal_slam::telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_EQ(summary.status, TransformLookupStatus::kTransformFromFuture);
  EXPECT_EQ(summary.reason, "tf_transform_from_future");
  EXPECT_DOUBLE_EQ(summary.transform_age_ms, -5.0);
}

}  // namespace
}  // namespace causal_slam::transform
