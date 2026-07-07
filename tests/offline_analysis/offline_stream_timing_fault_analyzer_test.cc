#include "application/offline_analysis/offline_stream_timing_fault_analyzer.h"

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

namespace causal_slam::offline_analysis {
namespace {

std::uint64_t FaultCount(const StreamTimingFaultReport& faults, const std::string& reason) {
  const auto it = faults.fault_reasons.find(reason);
  return it == faults.fault_reasons.end() ? 0 : it->second;
}

TEST(OfflineStreamTimingFaultAnalyzerTest, NoPeriodProducesNoFaults) {
  OfflineTemporalReport report;

  const auto faults = BuildStreamTimingFaultReport(report);

  EXPECT_FALSE(faults.lidar_stream_timing_jitter_high);
  EXPECT_FALSE(faults.lidar_stream_timing_short_period);
  EXPECT_FALSE(faults.lidar_stream_timing_long_period);
  EXPECT_TRUE(faults.fault_reasons.empty());
}

TEST(OfflineStreamTimingFaultAnalyzerTest, NtuLikeTimingProducesJitterAndShortPeriodFaults) {
  OfflineTemporalReport report;
  report.lidar_timing.has_period = true;
  report.lidar_timing.period_count = 10;
  report.lidar_timing.period_mean_ms = 100.0;
  report.lidar_timing.period_min_ms = 69.0;
  report.lidar_timing.period_max_ms = 108.0;
  report.lidar_timing.period_stddev_ms = 9.0;

  const auto faults = BuildStreamTimingFaultReport(report);

  EXPECT_TRUE(faults.lidar_stream_timing_jitter_high);
  EXPECT_TRUE(faults.lidar_stream_timing_short_period);
  EXPECT_FALSE(faults.lidar_stream_timing_long_period);

  EXPECT_EQ(FaultCount(faults, "lidar_stream_timing_jitter_high"), 1U);
  EXPECT_EQ(FaultCount(faults, "lidar_stream_timing_short_period"), 1U);
  EXPECT_EQ(FaultCount(faults, "lidar_stream_timing_long_period"), 0U);
}

TEST(OfflineStreamTimingFaultAnalyzerTest, LongPeriodProducesLongPeriodFault) {
  OfflineTemporalReport report;
  report.lidar_timing.has_period = true;
  report.lidar_timing.period_count = 10;
  report.lidar_timing.period_mean_ms = 100.0;
  report.lidar_timing.period_min_ms = 99.0;
  report.lidar_timing.period_max_ms = 130.0;
  report.lidar_timing.period_stddev_ms = 1.0;

  const auto faults = BuildStreamTimingFaultReport(report);

  EXPECT_FALSE(faults.lidar_stream_timing_jitter_high);
  EXPECT_FALSE(faults.lidar_stream_timing_short_period);
  EXPECT_TRUE(faults.lidar_stream_timing_long_period);

  EXPECT_EQ(FaultCount(faults, "lidar_stream_timing_long_period"), 1U);
}

}  // namespace
}  // namespace causal_slam::offline_analysis
