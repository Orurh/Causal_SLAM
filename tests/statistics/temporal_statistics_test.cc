#include "domain/statistics/temporal_statistics.h"

#include <algorithm>
#include <cstdint>

#include <gtest/gtest.h>

namespace causal_slam::statistics {
namespace {

namespace coverage = causal_slam::coverage;
namespace lidar = causal_slam::lidar;
namespace model = causal_slam::model;
namespace telemetry = causal_slam::telemetry;

std::int64_t Seconds(std::int64_t seconds) {
  return seconds * 1'000'000'000LL;
}

const StreamTimingStatistics* FindStream(
    const TemporalWindowStatistics& stats,
    telemetry::TemporalStreamId id) {
  const auto it = std::find_if(
      stats.streams.begin(), stats.streams.end(), [id](const auto& stream) {
        return stream.id == id;
      });

  if (it == stats.streams.end()) {
    return nullptr;
  }

  return &*it;
}

model::TemporalObservation MakeSnapshot(
    double lidar_delay_ms,
    lidar::LidarScanWindowSource source,
    lidar::LidarScanWindowConfidence confidence) {
  model::TemporalObservation snapshot;
  telemetry::TimingSummary imu_timing;
  imu_timing.total_count = 1;
  imu_timing.health = telemetry::TimingHealth::kOk;
  imu_timing.reason = "ok";
  imu_timing.last_delay_ms = 0.3;
  imu_timing.last_period_ms = 20.0;
  imu_timing.last_jitter_ms = 0.1;

  telemetry::TimingSummary lidar_timing;
  lidar_timing.total_count = 1;
  lidar_timing.health = telemetry::TimingHealth::kOk;
  lidar_timing.reason = "ok";
  lidar_timing.last_delay_ms = lidar_delay_ms;
  lidar_timing.last_period_ms = 100.0;
  lidar_timing.last_jitter_ms = 0.2;

  snapshot.streams = {
      telemetry::MakeStreamTimingDiagnostic(
          telemetry::TemporalStreamId::kImu, imu_timing),
      telemetry::MakeStreamTimingDiagnostic(
          telemetry::TemporalStreamId::kLidar, lidar_timing),
  };

  coverage::ImuCoverageSummary imu_coverage;
  imu_coverage.health = coverage::ImuCoverageHealth::kOk;
  imu_coverage.reason = "ok";
  imu_coverage.imu_count_in_window = 5;
  imu_coverage.coverage_ratio = 0.8;
  imu_coverage.max_gap_inside_ms = 20.0;
  snapshot.imu_coverage = imu_coverage;

  lidar::LidarScanWindowEstimate scan_window;
  scan_window.source = source;
  scan_window.confidence = confidence;
  scan_window.duration_ms = 100.0;
  scan_window.reason = "test";
  snapshot.lidar_scan_window = scan_window;

  return snapshot;
}

TEST(TemporalStatisticsAggregatorTest, EmptySnapshotHasZeroCounts) {
  const TemporalStatisticsAggregator aggregator;

  const auto snapshot = aggregator.Snapshot(Seconds(100));

  EXPECT_EQ(snapshot.short_window.sample_count, 0u);
  EXPECT_EQ(snapshot.medium_window.sample_count, 0u);
  EXPECT_EQ(snapshot.session.sample_count, 0u);
}

TEST(TemporalStatisticsAggregatorTest, ComputesNumericStats) {
  TemporalStatisticsAggregator aggregator;

  aggregator.Observe(
      Seconds(1),
      MakeSnapshot(1.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  aggregator.Observe(
      Seconds(2),
      MakeSnapshot(2.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  aggregator.Observe(
      Seconds(3),
      MakeSnapshot(3.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  aggregator.Observe(
      Seconds(4),
      MakeSnapshot(4.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  aggregator.Observe(
      Seconds(5),
      MakeSnapshot(100.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  const auto snapshot = aggregator.Snapshot(Seconds(5));

  const auto* lidar_stats = FindStream(snapshot.session, telemetry::TemporalStreamId::kLidar);

  ASSERT_NE(lidar_stats, nullptr);
  EXPECT_EQ(lidar_stats->delay_ms.count, 5u);
  EXPECT_DOUBLE_EQ(lidar_stats->delay_ms.min, 1.0);
  EXPECT_DOUBLE_EQ(lidar_stats->delay_ms.max, 100.0);
  EXPECT_DOUBLE_EQ(lidar_stats->delay_ms.mean, 22.0);
  EXPECT_DOUBLE_EQ(lidar_stats->delay_ms.median, 3.0);
  EXPECT_DOUBLE_EQ(lidar_stats->delay_ms.p95, 100.0);
}

TEST(TemporalStatisticsAggregatorTest, RollingWindowPrunesOldSamples) {
  TemporalStatisticsAggregatorConfig config;
  config.short_window_ns = Seconds(10);
  config.medium_window_ns = Seconds(60);

  TemporalStatisticsAggregator aggregator{config};

  aggregator.Observe(
      Seconds(0),
      MakeSnapshot(1.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  aggregator.Observe(
      Seconds(5),
      MakeSnapshot(2.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  aggregator.Observe(
      Seconds(20),
      MakeSnapshot(3.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  const auto snapshot = aggregator.Snapshot(Seconds(20));
  const auto* lidar_stats = FindStream(snapshot.short_window, telemetry::TemporalStreamId::kLidar);

  EXPECT_EQ(snapshot.short_window.sample_count, 1u);
  ASSERT_NE(lidar_stats, nullptr);
  EXPECT_DOUBLE_EQ(lidar_stats->delay_ms.mean, 3.0);

  EXPECT_EQ(snapshot.medium_window.sample_count, 3u);
  EXPECT_EQ(snapshot.session.sample_count, 3u);
}

TEST(TemporalStatisticsAggregatorTest, CountsHealthSourceAndConfidence) {
  TemporalStatisticsAggregator aggregator;

  aggregator.Observe(
      Seconds(1),
      MakeSnapshot(1.0,
                   lidar::LidarScanWindowSource::kPointTimeField,                   lidar::LidarScanWindowConfidence::kHigh),
      telemetry::TemporalHealthStatus::kOk);

  aggregator.Observe(
      Seconds(2),
      MakeSnapshot(2.0,
                   lidar::LidarScanWindowSource::kMeasuredHeaderPeriod,                   lidar::LidarScanWindowConfidence::kMedium),
      telemetry::TemporalHealthStatus::kWarning);

  aggregator.Observe(
      Seconds(3),
      MakeSnapshot(3.0,
                   lidar::LidarScanWindowSource::kAssumedFixedDuration,                   lidar::LidarScanWindowConfidence::kLow),
      telemetry::TemporalHealthStatus::kDegraded);

  const auto snapshot = aggregator.Snapshot(Seconds(3));

  EXPECT_EQ(snapshot.session.health.ok_count, 1u);
  EXPECT_EQ(snapshot.session.health.warning_count, 1u);
  EXPECT_EQ(snapshot.session.health.degraded_count, 1u);
  EXPECT_EQ(snapshot.session.health.invalid_count, 0u);

  EXPECT_EQ(snapshot.session.scan_window_sources.point_time_field_count, 1u);
  EXPECT_EQ(snapshot.session.scan_window_sources.measured_header_period_count,
            1u);
  EXPECT_EQ(snapshot.session.scan_window_sources.assumed_fixed_duration_count,
            1u);

  EXPECT_EQ(snapshot.session.scan_window_confidence.high_count, 1u);
  EXPECT_EQ(snapshot.session.scan_window_confidence.medium_count, 1u);
  EXPECT_EQ(snapshot.session.scan_window_confidence.low_count, 1u);
}

TEST(TemporalStatisticsAggregatorTest,
     MissingLidarDoesNotProduceZeroLidarStats) {
  TemporalStatisticsAggregator aggregator;

  auto snapshot = MakeSnapshot(
      0.0,
      lidar::LidarScanWindowSource::kAssumedFixedDuration,
      lidar::LidarScanWindowConfidence::kLow);

  bool disabled_lidar = false;
  for (auto& stream : snapshot.streams) {
    if (stream.id == telemetry::TemporalStreamId::kLidar) {
      stream.timing.total_count = 0;
      stream.timing.last_delay_ms = 0.0;
      stream.timing.last_period_ms = 0.0;
      stream.timing.last_jitter_ms = 0.0;
      disabled_lidar = true;
    }
  }

  ASSERT_TRUE(disabled_lidar);

  snapshot.imu_coverage.reset();
  snapshot.lidar_scan_window.reset();

  aggregator.Observe(Seconds(1), snapshot, telemetry::TemporalHealthStatus::kOk);

  const auto stats = aggregator.Snapshot(Seconds(1));
  const auto* imu_stats = FindStream(stats.session, telemetry::TemporalStreamId::kImu);
  const auto* lidar_stats = FindStream(stats.session, telemetry::TemporalStreamId::kLidar);

  EXPECT_EQ(stats.session.sample_count, 1u);

  EXPECT_EQ(lidar_stats, nullptr);

  EXPECT_EQ(stats.session.scan_window_sources.assumed_fixed_duration_count, 0u);
  EXPECT_EQ(stats.session.scan_window_confidence.low_count, 0u);

  ASSERT_NE(imu_stats, nullptr);
  EXPECT_EQ(imu_stats->period_ms.count, 1u);
  EXPECT_EQ(imu_stats->jitter_ms.count, 1u);
}

}  // namespace
}  // namespace causal_slam::statistics