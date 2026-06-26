#include "domain/sensors/lidar/lidar_scan_timing.h"

#include <cstdint>

#include <gtest/gtest.h>

namespace causal_slam::lidar {
namespace {

std::int64_t Ms(const std::int64_t milliseconds) {
  return milliseconds * 1'000'000;
}

TEST(LidarScanTimingTest, BuildsWindowFromScanStartStamp) {
  const auto window = BuildLidarScanWindow(Ms(1000), 100.0, LidarStampPolicy::kScanStart);

  EXPECT_EQ(window.start_ns, Ms(1000));
  EXPECT_EQ(window.end_ns, Ms(1100));
}

TEST(LidarScanTimingTest, BuildsWindowFromScanMiddleStamp) {
  const auto window = BuildLidarScanWindow(Ms(1000), 100.0, LidarStampPolicy::kScanMiddle);

  EXPECT_EQ(window.start_ns, Ms(950));
  EXPECT_EQ(window.end_ns, Ms(1050));
}

TEST(LidarScanTimingTest, BuildsWindowFromScanEndStamp) {
  const auto window = BuildLidarScanWindow(Ms(1000), 100.0, LidarStampPolicy::kScanEnd);

  EXPECT_EQ(window.start_ns, Ms(900));
  EXPECT_EQ(window.end_ns, Ms(1000));
}

}  // namespace
}  // namespace causal_slam::lidar