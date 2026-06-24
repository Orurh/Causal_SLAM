#include "application/temporal_monitor/temporal_monitor_runtime_defaults.h"

#include <gtest/gtest.h>

namespace causal_slam::config {
namespace {

TEST(TemporalMonitorRuntimeDefaultsTest, ProvidesStableApplicationDefaults) {
  const auto defaults = MakeDefaultTemporalMonitorRuntimeDefaults();

  EXPECT_EQ(defaults.imu_topic, "/imu/data");
  EXPECT_EQ(defaults.lidar_topic, "/points");

  EXPECT_DOUBLE_EQ(defaults.summary_period_ms, 2000.0);
  EXPECT_DOUBLE_EQ(defaults.expected_imu_period_ms, 5.0);
  EXPECT_DOUBLE_EQ(defaults.imu_buffer_retention_ms, 5000.0);

  EXPECT_DOUBLE_EQ(defaults.pipeline.imu_gap_threshold_ms, 100.0);
  EXPECT_DOUBLE_EQ(defaults.pipeline.lidar_gap_threshold_ms, 500.0);

  EXPECT_DOUBLE_EQ(
      defaults.pipeline.lidar_scan_window.fallback_scan_duration_ms, 100.0);
  EXPECT_DOUBLE_EQ(
      defaults.pipeline.transform_age.max_transform_age_ms, 50.0);
  EXPECT_DOUBLE_EQ(
      defaults.pipeline.transform_age.max_future_tolerance_ms, 1.0);

  ASSERT_EQ(defaults.tf_target_frames.size(), 1U);
  ASSERT_EQ(defaults.tf_source_frames.size(), 1U);
  EXPECT_EQ(defaults.tf_target_frames[0], "odom");
  EXPECT_EQ(defaults.tf_source_frames[0], "<cloud_frame>");
}

}  // namespace
}  // namespace causal_slam::config
