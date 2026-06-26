#include "domain/policy/lidar_cloud_gate.h"

#include <cstdint>

#include <gtest/gtest.h>

#include "domain/telemetry/temporal_health.h"

namespace causal_slam::policy {
namespace {

namespace telemetry = causal_slam::telemetry;

LidarCloudGateResult Evaluate(
    LidarGateMode mode,
    telemetry::TemporalHealthStatus health,
    std::uint64_t total_imu_samples = 5,
    std::uint64_t window_imu_samples = 2) {
  const LidarCloudGateConfig config{
      .mode = mode,
      .min_total_imu_samples_before_forward = 5,
      .min_window_imu_samples_before_forward = 2,
  };

  return EvaluateLidarCloudGate(
      config,
      LidarCloudGateInput{
          .health = health,
          .total_imu_samples = total_imu_samples,
          .window_imu_samples = window_imu_samples,
      });
}

TEST(LidarCloudGateTest, ParsesKnownModes) {
  EXPECT_EQ(ParseLidarGateMode("observe"), LidarGateMode::kObserve);
  EXPECT_EQ(ParseLidarGateMode("drop_invalid"), LidarGateMode::kDropInvalid);
  EXPECT_EQ(ParseLidarGateMode("drop_degraded"), LidarGateMode::kDropDegraded);
  EXPECT_EQ(ParseLidarGateMode("strict"), LidarGateMode::kStrict);
}

TEST(LidarCloudGateTest, UnknownModeFallsBackToObserve) {
  EXPECT_EQ(ParseLidarGateMode("bad_mode"), LidarGateMode::kObserve);
}

TEST(LidarCloudGateTest, ModeStringsAreStable) {
  EXPECT_STREQ(ToString(LidarGateMode::kObserve), "observe");
  EXPECT_STREQ(ToString(LidarGateMode::kDropInvalid), "drop_invalid");
  EXPECT_STREQ(ToString(LidarGateMode::kDropDegraded), "drop_degraded");
  EXPECT_STREQ(ToString(LidarGateMode::kStrict), "strict");
}

TEST(LidarCloudGateTest, ReasonStringsAreStable) {
  EXPECT_STREQ(ToString(LidarCloudGateReason::kObserveMode), "observe_mode");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kInsufficientTimingEvidence),
               "insufficient_timing_evidence");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthOk),
               "temporal_health_ok");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthWarning),
               "temporal_health_warning");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthDegraded),
               "temporal_health_degraded");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthInvalid),
               "temporal_health_invalid");
}

TEST(LidarCloudGateTest, ObserveModeAlwaysForwards) {
  const auto result =
      Evaluate(LidarGateMode::kObserve,
               telemetry::TemporalHealthStatus::kInvalid,
               0,
               0);

  EXPECT_TRUE(result.should_forward);
  EXPECT_EQ(result.reason, LidarCloudGateReason::kObserveMode);
}

TEST(LidarCloudGateTest, ActiveGateBlocksDuringWarmup) {
  const auto result =
      Evaluate(LidarGateMode::kStrict,
               telemetry::TemporalHealthStatus::kOk,
               4,
               2);

  EXPECT_FALSE(result.should_forward);
  EXPECT_EQ(result.reason, LidarCloudGateReason::kInsufficientTimingEvidence);
}

TEST(LidarCloudGateTest, ObserveModeIgnoresWarmup) {
  const auto result =
      Evaluate(LidarGateMode::kObserve,
               telemetry::TemporalHealthStatus::kInvalid,
               0,
               0);

  EXPECT_TRUE(result.should_forward);
  EXPECT_EQ(result.reason, LidarCloudGateReason::kObserveMode);
}

TEST(LidarCloudGateTest, DropInvalidRejectsOnlyInvalidHealth) {
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropInvalid,
                       telemetry::TemporalHealthStatus::kOk)
                  .should_forward);
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropInvalid,
                       telemetry::TemporalHealthStatus::kWarning)
                  .should_forward);
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropInvalid,
                       telemetry::TemporalHealthStatus::kDegraded)
                  .should_forward);

  const auto invalid =
      Evaluate(LidarGateMode::kDropInvalid,
               telemetry::TemporalHealthStatus::kInvalid);

  EXPECT_FALSE(invalid.should_forward);
  EXPECT_EQ(invalid.reason, LidarCloudGateReason::kTemporalHealthInvalid);
}

TEST(LidarCloudGateTest, DropDegradedAllowsOnlyOkAndWarning) {
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropDegraded,
                       telemetry::TemporalHealthStatus::kOk)
                  .should_forward);
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropDegraded,
                       telemetry::TemporalHealthStatus::kWarning)
                  .should_forward);

  EXPECT_FALSE(Evaluate(LidarGateMode::kDropDegraded,
                        telemetry::TemporalHealthStatus::kDegraded)
                   .should_forward);
  EXPECT_FALSE(Evaluate(LidarGateMode::kDropDegraded,
                        telemetry::TemporalHealthStatus::kInvalid)
                   .should_forward);
}

TEST(LidarCloudGateTest, StrictAllowsOnlyOk) {
  EXPECT_TRUE(Evaluate(LidarGateMode::kStrict,
                       telemetry::TemporalHealthStatus::kOk)
                  .should_forward);

  EXPECT_FALSE(Evaluate(LidarGateMode::kStrict,
                        telemetry::TemporalHealthStatus::kWarning)
                   .should_forward);
  EXPECT_FALSE(Evaluate(LidarGateMode::kStrict,
                        telemetry::TemporalHealthStatus::kDegraded)
                   .should_forward);
  EXPECT_FALSE(Evaluate(LidarGateMode::kStrict,
                        telemetry::TemporalHealthStatus::kInvalid)
                   .should_forward);
}

TEST(LidarCloudGateTest, DetectsMinimumTimingEvidence) {
  const LidarCloudGateConfig config{
      .mode = LidarGateMode::kDropInvalid,
      .min_total_imu_samples_before_forward = 5,
      .min_window_imu_samples_before_forward = 2,
  };

  EXPECT_FALSE(HasMinimumTimingEvidenceForActiveGate(
      config,
      LidarCloudGateInput{
          .health = telemetry::TemporalHealthStatus::kOk,
          .total_imu_samples = 4,
          .window_imu_samples = 2,
      }));

  EXPECT_FALSE(HasMinimumTimingEvidenceForActiveGate(
      config,
      LidarCloudGateInput{
          .health = telemetry::TemporalHealthStatus::kOk,
          .total_imu_samples = 5,
          .window_imu_samples = 1,
      }));

  EXPECT_TRUE(HasMinimumTimingEvidenceForActiveGate(
      config,
      LidarCloudGateInput{
          .health = telemetry::TemporalHealthStatus::kOk,
          .total_imu_samples = 5,
          .window_imu_samples = 2,
      }));
}

}  // namespace
}  // namespace causal_slam::policy