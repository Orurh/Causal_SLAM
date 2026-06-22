#include "pipeline/temporal_monitor_pipeline.h"

#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "pointcloud/point_cloud2_datatype.h"

namespace causal_slam::pipeline {
namespace {

constexpr std::int64_t kNanosecondsPerMillisecond = 1'000'000LL;

std::int64_t Ms(std::int64_t value) {
  return value * kNanosecondsPerMillisecond;
}

template <typename T>
void WriteValue(
    std::vector<std::uint8_t>* data,
    std::uint32_t point_step,
    std::uint32_t point_index,
    std::uint32_t field_offset,
    const T& value) {
  const std::size_t byte_offset =
      static_cast<std::size_t>(point_index) *
          static_cast<std::size_t>(point_step) +
      static_cast<std::size_t>(field_offset);

  std::memcpy(data->data() + byte_offset, &value, sizeof(T));
}

TemporalMonitorPipelineConfig MakeConfig() {
  TemporalMonitorPipelineConfig config;

  config.imu_gap_threshold_ms = 500.0;
  config.lidar_gap_threshold_ms = 1000.0;
  config.imu_buffer_retention_ns = Ms(5000);

  config.lidar_scan_window.fallback_scan_duration_ms = 100.0;
  config.lidar_scan_window.min_measured_scan_duration_ms = 1.0;
  config.lidar_scan_window.max_measured_scan_duration_ms = 500.0;
  config.lidar_scan_window.stamp_policy =
      causal_slam::lidar::LidarStampPolicy::kScanEnd;
  config.lidar_scan_window.prefer_measured_header_period = true;

  config.imu_coverage.max_missing_prefix_ms = 40.0;
  config.imu_coverage.max_missing_suffix_ms = 40.0;
  config.imu_coverage.max_internal_gap_ms = 100.0;

  return config;
}

LidarPipelineInput MakeLidarInput(
    std::int64_t header_stamp_ns,
    std::int64_t receive_time_ns) {
  constexpr std::uint32_t point_count = 3;
  constexpr std::uint32_t point_step = sizeof(std::uint32_t);

  static std::vector<std::uint8_t> data(point_count * point_step);
  data.assign(point_count * point_step, 0);

  WriteValue<std::uint32_t>(&data, point_step, 0, 0, 0U);
  WriteValue<std::uint32_t>(&data, point_step, 1, 0, 50'000'000U);
  WriteValue<std::uint32_t>(&data, point_step, 2, 0, 100'000'000U);

  return LidarPipelineInput{
      .header_stamp_ns = header_stamp_ns,
      .receive_time_ns = receive_time_ns,
      .fields = {
          causal_slam::pointcloud::PointCloud2FieldInfo{
              .name = "offset_time",
              .offset = 0,
              .datatype = causal_slam::pointcloud::kPointCloud2Uint32,
              .count = 1,
          },
      },
      .cloud_view = causal_slam::pointcloud::PointCloud2CloudView{
          .header_stamp_ns = header_stamp_ns,
          .width = point_count,
          .height = 1,
          .point_step = point_step,
          .data = data.data(),
          .data_size = data.size(),
      },
  };
}

TEST(TemporalMonitorPipelineTest, HealthyImuCoverageAllowsMapUpdate) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  for (std::int64_t t_ms = 1000; t_ms <= 1120; t_ms += 20) {
    pipeline.ObserveImu(ImuPipelineInput{
        .header_stamp_ns = Ms(t_ms),
        .receive_time_ns = Ms(t_ms),
    });
  }

  pipeline.ObserveLidar(MakeLidarInput(Ms(1000), Ms(1100)));

  const auto snapshot = pipeline.BuildSnapshot(Ms(1200));

  EXPECT_EQ(snapshot.diagnostics.overall_status,
            causal_slam::telemetry::TemporalHealthStatus::kOk);
  EXPECT_TRUE(snapshot.diagnostics.map_update_decision.map_update_allowed);
  EXPECT_EQ(snapshot.diagnostics.observation.imu_buffer_size, 7U);
  ASSERT_TRUE(snapshot.diagnostics.observation.imu_coverage.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.imu_coverage->health,
            causal_slam::coverage::ImuCoverageHealth::kOk);
  ASSERT_TRUE(snapshot.diagnostics.observation.lidar_scan_window.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.lidar_scan_window->source,
            causal_slam::lidar::LidarScanWindowSource::kPointTimeField);
}

TEST(TemporalMonitorPipelineTest, MissingImuCoverageRejectsMapUpdate) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  pipeline.ObserveLidar(MakeLidarInput(Ms(1000), Ms(1100)));

  const auto snapshot = pipeline.BuildSnapshot(Ms(1200));

  EXPECT_EQ(snapshot.diagnostics.overall_status,
            causal_slam::telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_FALSE(snapshot.diagnostics.map_update_decision.map_update_allowed);
  ASSERT_TRUE(snapshot.diagnostics.observation.imu_coverage.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.imu_coverage->health,
            causal_slam::coverage::ImuCoverageHealth::kDegraded);
}

}  // namespace
}  // namespace causal_slam::pipeline
