#include "domain/policy/lidar_cloud_gate.h"

#include <cstdint>

#include <gtest/gtest.h>

#include "domain/telemetry/temporal_health.h"

namespace causal_slam::policy {
namespace {

namespace telemetry = causal_slam::telemetry;

LidarCloudGateConfig MakeConfig(
    LidarGateMode mode,
    std::uint64_t min_total_imu_samples = 5,
    std::uint64_t min_window_imu_samples = 2) {
  LidarCloudGateConfig config;
  config.mode = mode;
  config.min_total_imu_samples_before_forward =
      min_total_imu_samples;
  config.min_window_imu_samples_before_forward =
      min_window_imu_samples;
  return config;
}

LidarCloudGateInput MakeInput(
    telemetry::TemporalHealthStatus health,
    std::uint64_t total_imu_samples,
    std::uint64_t window_imu_samples,
    bool has_hard_fusion_blocker = false) {
  LidarCloudGateInput input;
  input.health = health;
  input.total_imu_samples = total_imu_samples;
  input.window_imu_samples = window_imu_samples;
  input.has_hard_fusion_blocker = has_hard_fusion_blocker;
  return input;
}

LidarCloudGateResult Evaluate(LidarGateMode mode, telemetry::TemporalHealthStatus health, std::uint64_t total_imu_samples = 5,
                              std::uint64_t window_imu_samples = 2) {
  const auto config = MakeConfig(mode);
  const auto input =
      MakeInput(health, total_imu_samples, window_imu_samples);
  return EvaluateLidarCloudGate(config, input);
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
  EXPECT_STREQ(ToString(LidarCloudGateReason::kInsufficientTimingEvidence), "insufficient_timing_evidence");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthOk), "temporal_health_ok");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthWarning), "temporal_health_warning");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthDegraded), "temporal_health_degraded");
  EXPECT_STREQ(ToString(LidarCloudGateReason::kTemporalHealthInvalid), "temporal_health_invalid");
}

TEST(LidarCloudGateTest, ObserveModeAlwaysForwards) {
  const auto result = Evaluate(LidarGateMode::kObserve, telemetry::TemporalHealthStatus::kInvalid, 0, 0);

  EXPECT_TRUE(result.should_forward);
  EXPECT_EQ(result.reason, LidarCloudGateReason::kObserveMode);
}

TEST(LidarCloudGateTest, ActiveGateBlocksDuringWarmup) {
  const auto result = Evaluate(LidarGateMode::kStrict, telemetry::TemporalHealthStatus::kOk, 4, 2);

  EXPECT_FALSE(result.should_forward);
  EXPECT_EQ(result.reason, LidarCloudGateReason::kInsufficientTimingEvidence);
}

TEST(LidarCloudGateTest, ObserveModeIgnoresWarmup) {
  const auto result = Evaluate(LidarGateMode::kObserve, telemetry::TemporalHealthStatus::kInvalid, 0, 0);

  EXPECT_TRUE(result.should_forward);
  EXPECT_EQ(result.reason, LidarCloudGateReason::kObserveMode);
}

TEST(LidarCloudGateTest, DropInvalidRejectsOnlyInvalidHealth) {
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropInvalid, telemetry::TemporalHealthStatus::kOk).should_forward);
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropInvalid, telemetry::TemporalHealthStatus::kWarning).should_forward);
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropInvalid, telemetry::TemporalHealthStatus::kDegraded).should_forward);

  const auto invalid = Evaluate(LidarGateMode::kDropInvalid, telemetry::TemporalHealthStatus::kInvalid);

  EXPECT_FALSE(invalid.should_forward);
  EXPECT_EQ(invalid.reason, LidarCloudGateReason::kTemporalHealthInvalid);
}

TEST(LidarCloudGateTest, DropDegradedForwardsDiagnosticDegradedWithoutHardFusionBlocker) {
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropDegraded, telemetry::TemporalHealthStatus::kOk).should_forward);
  EXPECT_TRUE(Evaluate(LidarGateMode::kDropDegraded, telemetry::TemporalHealthStatus::kWarning).should_forward);

  EXPECT_TRUE(Evaluate(LidarGateMode::kDropDegraded, telemetry::TemporalHealthStatus::kDegraded).should_forward);
  EXPECT_FALSE(Evaluate(LidarGateMode::kDropDegraded, telemetry::TemporalHealthStatus::kInvalid).should_forward);
}

TEST(LidarCloudGateTest, StrictAllowsOnlyOk) {
  EXPECT_TRUE(Evaluate(LidarGateMode::kStrict, telemetry::TemporalHealthStatus::kOk).should_forward);

  EXPECT_FALSE(Evaluate(LidarGateMode::kStrict, telemetry::TemporalHealthStatus::kWarning).should_forward);
  EXPECT_FALSE(Evaluate(LidarGateMode::kStrict, telemetry::TemporalHealthStatus::kDegraded).should_forward);
  EXPECT_FALSE(Evaluate(LidarGateMode::kStrict, telemetry::TemporalHealthStatus::kInvalid).should_forward);
}

TEST(LidarCloudGateTest, DetectsMinimumTimingEvidence) {
  const auto config = MakeConfig(LidarGateMode::kDropInvalid);

  EXPECT_FALSE(HasMinimumTimingEvidenceForActiveGate(
      config,
      MakeInput(telemetry::TemporalHealthStatus::kOk, 4, 2)));

  EXPECT_FALSE(HasMinimumTimingEvidenceForActiveGate(
      config,
      MakeInput(telemetry::TemporalHealthStatus::kOk, 5, 1)));

  EXPECT_TRUE(HasMinimumTimingEvidenceForActiveGate(
      config,
      MakeInput(telemetry::TemporalHealthStatus::kOk, 5, 2)));
}

}  // namespace

TEST(LidarCloudGateTest, ClassifiesHardFusionBlockingReasons) {
  EXPECT_TRUE(IsHardFusionBlockingReason("imu_window_incomplete"));
  EXPECT_TRUE(IsHardFusionBlockingReason("imu_window_empty"));
  EXPECT_TRUE(IsHardFusionBlockingReason("imu_window_missing_prefix"));
  EXPECT_TRUE(IsHardFusionBlockingReason("imu_window_missing_suffix"));
  EXPECT_TRUE(IsHardFusionBlockingReason("imu_window_internal_gap"));
  EXPECT_TRUE(IsHardFusionBlockingReason("invalid_scan_window"));
  EXPECT_TRUE(IsHardFusionBlockingReason("message_reordering_detected"));

  EXPECT_FALSE(IsHardFusionBlockingReason("lidar_stream_timing_jitter_high"));
  EXPECT_FALSE(IsHardFusionBlockingReason("imu_stream_timing_jitter_suspicious"));
  EXPECT_FALSE(IsHardFusionBlockingReason("none"));
}

TEST(LidarCloudGateTest, DropDegradedBlocksHardFusionBlocker) {
  const auto config =
      MakeConfig(LidarGateMode::kDropDegraded, 0, 0);
  const auto input = MakeInput(
      telemetry::TemporalHealthStatus::kDegraded,
      10,
      10,
      true);

  const auto result = EvaluateLidarCloudGate(config, input);

  EXPECT_FALSE(result.should_forward);
}

}  // namespace causal_slam::policy
