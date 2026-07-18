#include "domain/sensors/imu/imu_coverage_analyzer.h"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace causal_slam::coverage {
namespace {

std::int64_t Ms(const std::int64_t milliseconds) {
  return milliseconds * 1'000'000;
}

ImuSample ImuAtMs(const std::int64_t timestamp_ms) {
  ImuSample sample;
  sample.stamp_ns = Ms(timestamp_ms);
  return sample;
}

causal_slam::core::TimeWindow WindowMs(
    const std::int64_t start_ms,
    const std::int64_t end_ms) {
  causal_slam::core::TimeWindow window;
  window.start_ns = Ms(start_ms);
  window.end_ns = Ms(end_ms);
  return window;
}

TEST(ImuCoverageAnalyzerTest, FullCoverageIsOk) {
  ImuCoverageConfig config;
  config.max_missing_prefix_ms = 5.0;
  config.max_missing_suffix_ms = 5.0;
  config.max_internal_gap_ms = 30.0;
  const ImuCoverageAnalyzer analyzer{config};

  const std::vector<ImuSample> samples{
      ImuAtMs(1000), ImuAtMs(1020), ImuAtMs(1040), ImuAtMs(1060), ImuAtMs(1080), ImuAtMs(1100),
  };

  const ImuCoverageSummary summary = analyzer.Analyze(WindowMs(1000, 1100), samples);

  EXPECT_EQ(summary.imu_count_in_window, 6u);
  EXPECT_NEAR(summary.missing_prefix_ms, 0.0, 1e-9);
  EXPECT_NEAR(summary.missing_suffix_ms, 0.0, 1e-9);
  EXPECT_NEAR(summary.max_gap_inside_ms, 20.0, 1e-9);
  EXPECT_NEAR(summary.coverage_ratio, 1.0, 1e-9);
  EXPECT_EQ(summary.health, ImuCoverageHealth::kOk);
  EXPECT_EQ(summary.reason, "ok");
}

TEST(ImuCoverageAnalyzerTest, MissingPrefixDegradesCoverage) {
  const ImuCoverageAnalyzer analyzer{};

  const std::vector<ImuSample> samples{
      ImuAtMs(1030), ImuAtMs(1050), ImuAtMs(1070), ImuAtMs(1090), ImuAtMs(1100),
  };

  const ImuCoverageSummary summary = analyzer.Analyze(WindowMs(1000, 1100), samples);

  EXPECT_EQ(summary.imu_count_in_window, 5u);
  EXPECT_NEAR(summary.missing_prefix_ms, 30.0, 1e-9);
  EXPECT_EQ(summary.health, ImuCoverageHealth::kDegraded);
  EXPECT_EQ(summary.reason, "imu_window_missing_prefix");
}

TEST(ImuCoverageAnalyzerTest, MissingSuffixDegradesCoverage) {
  const ImuCoverageAnalyzer analyzer{};

  const std::vector<ImuSample> samples{
      ImuAtMs(1000), ImuAtMs(1020), ImuAtMs(1040), ImuAtMs(1060), ImuAtMs(1070),
  };

  const ImuCoverageSummary summary = analyzer.Analyze(WindowMs(1000, 1100), samples);

  EXPECT_EQ(summary.imu_count_in_window, 5u);
  EXPECT_NEAR(summary.missing_suffix_ms, 30.0, 1e-9);
  EXPECT_EQ(summary.health, ImuCoverageHealth::kDegraded);
  EXPECT_EQ(summary.reason, "imu_window_missing_suffix");
}

TEST(ImuCoverageAnalyzerTest, InternalGapDegradesCoverage) {
  ImuCoverageConfig config;
  config.max_missing_prefix_ms = 5.0;
  config.max_missing_suffix_ms = 5.0;
  config.max_internal_gap_ms = 30.0;
  const ImuCoverageAnalyzer analyzer{config};

  const std::vector<ImuSample> samples{
      ImuAtMs(1000),
      ImuAtMs(1020),
      ImuAtMs(1080),
      ImuAtMs(1100),
  };

  const ImuCoverageSummary summary = analyzer.Analyze(WindowMs(1000, 1100), samples);

  EXPECT_EQ(summary.imu_count_in_window, 4u);
  EXPECT_NEAR(summary.max_gap_inside_ms, 60.0, 1e-9);
  EXPECT_EQ(summary.health, ImuCoverageHealth::kDegraded);
  EXPECT_EQ(summary.reason, "imu_window_internal_gap");
}

TEST(ImuCoverageAnalyzerTest, EmptyWindowDegradesCoverage) {
  const ImuCoverageAnalyzer analyzer{};

  const std::vector<ImuSample> samples{
      ImuAtMs(900),
      ImuAtMs(950),
      ImuAtMs(1200),
  };

  const ImuCoverageSummary summary = analyzer.Analyze(WindowMs(1000, 1100), samples);

  EXPECT_EQ(summary.imu_count_in_window, 0u);
  EXPECT_NEAR(summary.coverage_ratio, 0.0, 1e-9);
  EXPECT_EQ(summary.health, ImuCoverageHealth::kDegraded);
  EXPECT_EQ(summary.reason, "imu_window_empty");
}

TEST(ImuCoverageAnalyzerTest, InvalidScanWindowDegradesCoverage) {
  const ImuCoverageAnalyzer analyzer{};

  const std::vector<ImuSample> samples{
      ImuAtMs(1000),
  };

  const ImuCoverageSummary summary = analyzer.Analyze(WindowMs(1100, 1000), samples);

  EXPECT_EQ(summary.health, ImuCoverageHealth::kDegraded);
  EXPECT_EQ(summary.reason, "invalid_scan_window");
}

}  // namespace
}  // namespace causal_slam::coverage