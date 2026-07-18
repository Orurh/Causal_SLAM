#include "application/temporal_monitor/temporal_monitor_pipeline.h"

#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "domain/sensors/pointcloud/point_cloud2_datatype.h"
#include "domain/time/time_window.h"

namespace causal_slam::pipeline {
namespace {

constexpr std::int64_t kNanosecondsPerMillisecond = 1'000'000LL;

std::int64_t Ms(std::int64_t value) {
  return value * kNanosecondsPerMillisecond;
}

template <typename T>
void WriteValue(std::vector<std::uint8_t>* data, std::uint32_t point_step, std::uint32_t point_index, std::uint32_t field_offset,
                const T& value) {
  const std::size_t byte_offset =
      static_cast<std::size_t>(point_index) * static_cast<std::size_t>(point_step) + static_cast<std::size_t>(field_offset);

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
  config.lidar_scan_window.stamp_policy = causal_slam::lidar::LidarStampPolicy::kScanEnd;
  config.lidar_scan_window.prefer_measured_header_period = true;

  config.imu_coverage.max_missing_prefix_ms = 40.0;
  config.imu_coverage.max_missing_suffix_ms = 40.0;
  config.imu_coverage.max_internal_gap_ms = 100.0;

  return config;
}

causal_slam::lidar::LidarScanWindowEstimate MakePrecomputedScanWindowEstimate(std::int64_t start_ns, std::int64_t duration_ms) {
  causal_slam::core::TimeWindow window;
  window.start_ns = start_ns;
  window.end_ns = start_ns + Ms(duration_ms);

  causal_slam::lidar::LidarScanWindowEstimate estimate;
  estimate.window = window;
  estimate.duration_ms = static_cast<double>(duration_ms);
  estimate.source =
      causal_slam::lidar::LidarScanWindowSource::kPointTimeField;
  estimate.confidence =
      causal_slam::lidar::LidarScanWindowConfidence::kHigh;
  estimate.reason = "precomputed_test_scan_window";
  return estimate;
}

LidarPipelineInput MakeLidarInput(std::int64_t header_stamp_ns, std::int64_t receive_time_ns) {
  constexpr std::uint32_t point_count = 3;
  constexpr std::uint32_t point_step = sizeof(std::uint32_t);

  static std::vector<std::uint8_t> data(point_count * point_step);
  data.assign(point_count * point_step, 0);

  WriteValue<std::uint32_t>(&data, point_step, 0, 0, 0U);
  WriteValue<std::uint32_t>(&data, point_step, 1, 0, 50'000'000U);
  WriteValue<std::uint32_t>(&data, point_step, 2, 0, 100'000'000U);

  causal_slam::pointcloud::PointCloud2FieldInfo field;
  field.name = "offset_time";
  field.offset = 0;
  field.datatype = causal_slam::pointcloud::kPointCloud2Uint32;
  field.count = 1;

  causal_slam::pointcloud::PointCloud2CloudView cloud_view;
  cloud_view.header_stamp_ns = header_stamp_ns;
  cloud_view.width = point_count;
  cloud_view.height = 1;
  cloud_view.point_step = point_step;
  cloud_view.data = data.data();
  cloud_view.data_size = data.size();

  LidarPipelineInput input;
  input.header_stamp_ns = header_stamp_ns;
  input.receive_time_ns = receive_time_ns;
  input.frame_id = "lidar";
  input.fields.push_back(field);
  input.cloud_view = cloud_view;
  input.precomputed_scan_window_estimate = std::nullopt;
  return input;
}

TEST(TemporalMonitorPipelineTest, HealthyImuCoverageAllowsMapUpdate) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  for (std::int64_t t_ms = 1000; t_ms <= 1120; t_ms += 20) {
    ImuPipelineInput imu_input;
    imu_input.header_stamp_ns = Ms(t_ms);
    imu_input.receive_time_ns = Ms(t_ms);
    pipeline.ObserveImu(imu_input);
  }

  pipeline.ObserveLidar(MakeLidarInput(Ms(1000), Ms(1100)));

  const auto snapshot = pipeline.BuildSnapshot(Ms(1200));

  EXPECT_EQ(snapshot.diagnostics.overall_status, causal_slam::telemetry::TemporalHealthStatus::kOk);
  EXPECT_TRUE(snapshot.map_update_decision.map_update_allowed);
  EXPECT_EQ(snapshot.diagnostics.observation.imu_buffer_size, 7U);
  ASSERT_TRUE(snapshot.diagnostics.observation.imu_coverage.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.imu_coverage->health, causal_slam::coverage::ImuCoverageHealth::kOk);
  ASSERT_TRUE(snapshot.diagnostics.observation.lidar_scan_window.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.lidar_scan_window->source, causal_slam::lidar::LidarScanWindowSource::kPointTimeField);
}

TEST(TemporalMonitorPipelineTest, UsesPrecomputedScanWindowEstimate) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  for (std::int64_t t_ms = 1000; t_ms <= 1180; t_ms += 20) {
    ImuPipelineInput imu_input;
    imu_input.header_stamp_ns = Ms(t_ms);
    imu_input.receive_time_ns = Ms(t_ms);
    pipeline.ObserveImu(imu_input);
  }

  auto lidar_input = MakeLidarInput(Ms(1000), Ms(1100));
  lidar_input.precomputed_scan_window_estimate = MakePrecomputedScanWindowEstimate(Ms(1000), 156);

  pipeline.ObserveLidar(lidar_input);

  const auto snapshot = pipeline.BuildSnapshot(Ms(1200));

  EXPECT_EQ(snapshot.diagnostics.overall_status, causal_slam::telemetry::TemporalHealthStatus::kOk);
  EXPECT_TRUE(snapshot.map_update_decision.map_update_allowed);

  ASSERT_TRUE(snapshot.diagnostics.observation.lidar_scan_window.has_value());
  const auto& scan_window = *snapshot.diagnostics.observation.lidar_scan_window;

  EXPECT_EQ(scan_window.window.start_ns, Ms(1000));
  EXPECT_EQ(scan_window.window.end_ns, Ms(1156));
  EXPECT_DOUBLE_EQ(scan_window.duration_ms, 156.0);
  EXPECT_EQ(scan_window.source, causal_slam::lidar::LidarScanWindowSource::kPointTimeField);
  EXPECT_EQ(scan_window.confidence, causal_slam::lidar::LidarScanWindowConfidence::kHigh);
  EXPECT_EQ(scan_window.reason, "precomputed_test_scan_window");

  ASSERT_TRUE(snapshot.diagnostics.observation.imu_coverage.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.imu_coverage->health, causal_slam::coverage::ImuCoverageHealth::kOk);
}

TEST(TemporalMonitorPipelineTest, MissingImuCoverageRejectsMapUpdate) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  pipeline.ObserveLidar(MakeLidarInput(Ms(1000), Ms(1100)));

  const auto snapshot = pipeline.BuildSnapshot(Ms(1200));

  EXPECT_EQ(snapshot.diagnostics.overall_status, causal_slam::telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_FALSE(snapshot.map_update_decision.map_update_allowed);
  ASSERT_TRUE(snapshot.diagnostics.observation.imu_coverage.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.imu_coverage->health, causal_slam::coverage::ImuCoverageHealth::kDegraded);
}

TEST(TemporalMonitorPipelineTest, ExtractsPointTimeWhenPrecomputedScanWindowIsMissing) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  for (std::int64_t t_ms = 1000; t_ms <= 1120; t_ms += 20) {
    ImuPipelineInput imu_input;
    imu_input.header_stamp_ns = Ms(t_ms);
    imu_input.receive_time_ns = Ms(t_ms);
    pipeline.ObserveImu(imu_input);
  }

  auto lidar_input = MakeLidarInput(Ms(1000), Ms(1100));
  lidar_input.precomputed_scan_window_estimate = std::nullopt;

  pipeline.ObserveLidar(lidar_input);

  const auto snapshot = pipeline.BuildSnapshot(Ms(1200));

  ASSERT_TRUE(snapshot.diagnostics.observation.lidar_scan_window.has_value());
  const auto& scan_window = *snapshot.diagnostics.observation.lidar_scan_window;

  EXPECT_EQ(scan_window.window.start_ns, Ms(1000));
  EXPECT_EQ(scan_window.window.end_ns, Ms(1100));
  EXPECT_DOUBLE_EQ(scan_window.duration_ms, 100.0);
  EXPECT_EQ(scan_window.source, causal_slam::lidar::LidarScanWindowSource::kPointTimeField);

  ASSERT_TRUE(snapshot.diagnostics.observation.imu_coverage.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.imu_coverage->health, causal_slam::coverage::ImuCoverageHealth::kOk);
}

TEST(TemporalMonitorPipelineTest, PrecomputedScanWindowControlsImuCoverageWindow) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  for (std::int64_t t_ms = 1000; t_ms <= 1200; t_ms += 20) {
    ImuPipelineInput imu_input;
    imu_input.header_stamp_ns = Ms(t_ms);
    imu_input.receive_time_ns = Ms(t_ms);
    pipeline.ObserveImu(imu_input);
  }

  auto lidar_input = MakeLidarInput(Ms(1000), Ms(1100));
  lidar_input.precomputed_scan_window_estimate = MakePrecomputedScanWindowEstimate(Ms(1000), 200);

  pipeline.ObserveLidar(lidar_input);

  const auto snapshot = pipeline.BuildSnapshot(Ms(1250));

  ASSERT_TRUE(snapshot.diagnostics.observation.lidar_scan_window.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.lidar_scan_window->window.end_ns, Ms(1200));
  EXPECT_DOUBLE_EQ(snapshot.diagnostics.observation.lidar_scan_window->duration_ms, 200.0);

  ASSERT_TRUE(snapshot.diagnostics.observation.imu_coverage.has_value());
  EXPECT_EQ(snapshot.diagnostics.observation.imu_coverage->health, causal_slam::coverage::ImuCoverageHealth::kOk);
}

TEST(TemporalMonitorPipelineTest, TransformObservationIsStoredInSnapshot) {
  TemporalMonitorPipeline pipeline{MakeConfig()};

  causal_slam::transform::TransformLookupObservation observation;
  observation.target_frame = "odom";
  observation.source_frame = "base_link";
  observation.requested_stamp_ns = Ms(1000);
  observation.transform_stamp_ns = Ms(990);
  observation.receive_time_ns = Ms(1010);
  observation.lookup_success = true;
  observation.extrapolation_required = false;

  pipeline.ObserveTransform(observation);

  const auto snapshot = pipeline.BuildSnapshot(Ms(1200));

  ASSERT_FALSE(snapshot.diagnostics.observation.transform_ages.empty());
  EXPECT_EQ(snapshot.diagnostics.observation.transform_ages.front().status, causal_slam::transform::TransformLookupStatus::kOk);
  EXPECT_EQ(snapshot.diagnostics.observation.transform_ages.front().target_frame, "odom");
  EXPECT_EQ(snapshot.diagnostics.observation.transform_ages.front().source_frame, "base_link");
  EXPECT_DOUBLE_EQ(snapshot.diagnostics.observation.transform_ages.front().transform_age_ms, 10.0);
}

}  // namespace
}  // namespace causal_slam::pipeline
