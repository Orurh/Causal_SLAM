#include "domain/sensors/imu/imu_coverage_edge_tolerance.h"

#include <limits>
#include <optional>

#include <gtest/gtest.h>

namespace causal_slam::coverage {
namespace {

TEST(ImuCoverageEdgeToleranceTest, UsesConfiguredMinimumWithoutObservation) {
  ImuCoverageEdgeToleranceConfig config;
  config.configured_min_tolerance_ms = 5.0;
  config.observed_period_multiplier = 1.5;

  const auto result = ResolveImuCoverageEdgeTolerance(config, std::nullopt);

  EXPECT_DOUBLE_EQ(result.effective_tolerance_ms, 5.0);
  EXPECT_EQ(result.source, ImuCoverageEdgeToleranceSource::kConfiguredMinimum);
}

TEST(ImuCoverageEdgeToleranceTest, AdaptsToObservedTwoHundredHertzImu) {
  ImuCoverageEdgeToleranceConfig config;
  config.configured_min_tolerance_ms = 5.0;
  config.observed_period_multiplier = 1.5;

  const auto result = ResolveImuCoverageEdgeTolerance(config, 5.0);

  EXPECT_DOUBLE_EQ(result.adaptive_tolerance_ms, 7.5);
  EXPECT_DOUBLE_EQ(result.effective_tolerance_ms, 7.5);
  EXPECT_EQ(result.source, ImuCoverageEdgeToleranceSource::kObservedPeriodP95);
}

TEST(ImuCoverageEdgeToleranceTest, DoesNotReduceConfiguredMinimum) {
  ImuCoverageEdgeToleranceConfig config;
  config.configured_min_tolerance_ms = 5.0;
  config.observed_period_multiplier = 1.5;

  const auto result = ResolveImuCoverageEdgeTolerance(config, 2.5);

  EXPECT_DOUBLE_EQ(result.adaptive_tolerance_ms, 3.75);
  EXPECT_DOUBLE_EQ(result.effective_tolerance_ms, 5.0);
  EXPECT_EQ(result.source, ImuCoverageEdgeToleranceSource::kConfiguredMinimum);
}

TEST(ImuCoverageEdgeToleranceTest, IgnoresInvalidObservedPeriod) {
  ImuCoverageEdgeToleranceConfig config;
  config.configured_min_tolerance_ms = 5.0;
  config.observed_period_multiplier = 1.5;

  const auto result = ResolveImuCoverageEdgeTolerance(config, std::numeric_limits<double>::quiet_NaN());

  EXPECT_DOUBLE_EQ(result.effective_tolerance_ms, 5.0);
  EXPECT_DOUBLE_EQ(result.adaptive_tolerance_ms, 0.0);
  EXPECT_EQ(result.source, ImuCoverageEdgeToleranceSource::kConfiguredMinimum);
}

TEST(ImuCoverageEdgeToleranceTest, RendersStableSourceNames) {
  EXPECT_STREQ(ToString(ImuCoverageEdgeToleranceSource::kConfiguredMinimum), "configured_minimum");
  EXPECT_STREQ(ToString(ImuCoverageEdgeToleranceSource::kObservedPeriodP95), "observed_period_p95");
}

}  // namespace
}  // namespace causal_slam::coverage
