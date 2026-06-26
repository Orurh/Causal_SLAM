#include "domain/sensors/lidar/lidar_scan_window_estimator.h"

#include <cstdint>

#include <gtest/gtest.h>

namespace causal_slam::lidar {
namespace {

std::int64_t Ms(std::int64_t milliseconds) {
  return milliseconds * 1'000'000;
}

TEST(LidarScanWindowEstimatorTest, FirstScanUsesFallbackDuration) {
  LidarScanWindowEstimator estimator{LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = 100.0,
      .stamp_policy = LidarStampPolicy::kScanEnd,
  }};

  const auto estimate = estimator.Estimate(Ms(1000));

  EXPECT_EQ(estimate.window.start_ns, Ms(900));
  EXPECT_EQ(estimate.window.end_ns, Ms(1000));
  EXPECT_DOUBLE_EQ(estimate.duration_ms, 100.0);
  EXPECT_EQ(estimate.source, LidarScanWindowSource::kAssumedFixedDuration);
  EXPECT_EQ(estimate.confidence, LidarScanWindowConfidence::kLow);
  EXPECT_EQ(estimate.reason, "no_previous_lidar_stamp");
}

TEST(LidarScanWindowEstimatorTest, SecondScanUsesMeasuredHeaderPeriod) {
  LidarScanWindowEstimator estimator{LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = 100.0,
      .stamp_policy = LidarStampPolicy::kScanEnd,
  }};

  (void)estimator.Estimate(Ms(1000));
  const auto estimate = estimator.Estimate(Ms(1110));

  EXPECT_EQ(estimate.window.start_ns, Ms(1000));
  EXPECT_EQ(estimate.window.end_ns, Ms(1110));
  EXPECT_DOUBLE_EQ(estimate.duration_ms, 110.0);
  EXPECT_EQ(estimate.source, LidarScanWindowSource::kMeasuredHeaderPeriod);
  EXPECT_EQ(estimate.confidence, LidarScanWindowConfidence::kMedium);
  EXPECT_EQ(estimate.reason, "measured_header_period");
}

TEST(LidarScanWindowEstimatorTest, ReorderedStampFallsBackToFixedDuration) {
  LidarScanWindowEstimator estimator{LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = 100.0,
      .stamp_policy = LidarStampPolicy::kScanEnd,
  }};

  (void)estimator.Estimate(Ms(1000));
  const auto estimate = estimator.Estimate(Ms(990));

  EXPECT_EQ(estimate.window.start_ns, Ms(890));
  EXPECT_EQ(estimate.window.end_ns, Ms(990));
  EXPECT_DOUBLE_EQ(estimate.duration_ms, 100.0);
  EXPECT_EQ(estimate.source, LidarScanWindowSource::kAssumedFixedDuration);
  EXPECT_EQ(estimate.confidence, LidarScanWindowConfidence::kLow);
  EXPECT_EQ(estimate.reason, "invalid_measured_header_period");
}

TEST(LidarScanWindowEstimatorTest, OutOfRangeMeasuredPeriodFallsBack) {
  LidarScanWindowEstimator estimator{LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = 100.0,
      .min_measured_scan_duration_ms = 10.0,
      .max_measured_scan_duration_ms = 200.0,
      .stamp_policy = LidarStampPolicy::kScanEnd,
  }};

  (void)estimator.Estimate(Ms(1000));
  const auto estimate = estimator.Estimate(Ms(1500));

  EXPECT_EQ(estimate.window.start_ns, Ms(1400));
  EXPECT_EQ(estimate.window.end_ns, Ms(1500));
  EXPECT_DOUBLE_EQ(estimate.duration_ms, 100.0);
  EXPECT_EQ(estimate.source, LidarScanWindowSource::kAssumedFixedDuration);
  EXPECT_EQ(estimate.confidence, LidarScanWindowConfidence::kLow);
  EXPECT_EQ(estimate.reason, "measured_header_period_out_of_range");
}

TEST(LidarScanWindowEstimatorTest, MeasuredPeriodSupportsScanMiddlePolicy) {
  LidarScanWindowEstimator estimator{LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = 100.0,
      .stamp_policy = LidarStampPolicy::kScanMiddle,
  }};

  (void)estimator.Estimate(Ms(1000));
  const auto estimate = estimator.Estimate(Ms(1100));

  EXPECT_EQ(estimate.window.start_ns, Ms(1050));
  EXPECT_EQ(estimate.window.end_ns, Ms(1150));
  EXPECT_DOUBLE_EQ(estimate.duration_ms, 100.0);
  EXPECT_EQ(estimate.source, LidarScanWindowSource::kMeasuredHeaderPeriod);
  EXPECT_EQ(estimate.confidence, LidarScanWindowConfidence::kMedium);
}

TEST(LidarScanWindowEstimatorTest, CanDisableMeasuredHeaderPeriod) {
  LidarScanWindowEstimator estimator{LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = 100.0,
      .stamp_policy = LidarStampPolicy::kScanEnd,
      .prefer_measured_header_period = false,
  }};

  (void)estimator.Estimate(Ms(1000));
  const auto estimate = estimator.Estimate(Ms(1200));

  EXPECT_EQ(estimate.window.start_ns, Ms(1100));
  EXPECT_EQ(estimate.window.end_ns, Ms(1200));
  EXPECT_DOUBLE_EQ(estimate.duration_ms, 100.0);
  EXPECT_EQ(estimate.source, LidarScanWindowSource::kAssumedFixedDuration);
  EXPECT_EQ(estimate.confidence, LidarScanWindowConfidence::kLow);
  EXPECT_EQ(estimate.reason, "measured_header_period_disabled");
}

}  // namespace
}  // namespace causal_slam::lidar