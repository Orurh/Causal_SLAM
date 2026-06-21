#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "telemetry/temporal_health.h"
#include "model/temporal_observation.h"

namespace causal_slam::statistics {

struct NumericStats {
  std::uint64_t count{0};

  double min{0.0};
  double max{0.0};
  double mean{0.0};
  double median{0.0};
  double p95{0.0};
};

struct HealthDistribution {
  std::uint64_t ok_count{0};
  std::uint64_t warning_count{0};
  std::uint64_t degraded_count{0};
};

struct LidarScanWindowSourceDistribution {
  std::uint64_t assumed_fixed_duration_count{0};
  std::uint64_t measured_header_period_count{0};
  std::uint64_t point_time_field_count{0};
  std::uint64_t driver_metadata_count{0};
};

struct LidarScanWindowConfidenceDistribution {
  std::uint64_t low_count{0};
  std::uint64_t medium_count{0};
  std::uint64_t high_count{0};
};

struct StreamTimingStatistics {
  causal_slam::telemetry::TemporalStreamId id{
      causal_slam::telemetry::TemporalStreamId::kUnknown};

  NumericStats delay_ms;
  NumericStats period_ms;
  NumericStats jitter_ms;
};

struct TemporalWindowStatistics {
  std::int64_t window_duration_ns{0};
  std::uint64_t sample_count{0};

  HealthDistribution health;

  std::vector<StreamTimingStatistics> streams;

  NumericStats imu_coverage_ratio;
  NumericStats imu_samples_in_window;
  NumericStats imu_max_gap_inside_ms;

  LidarScanWindowSourceDistribution scan_window_sources;
  LidarScanWindowConfidenceDistribution scan_window_confidence;
};

struct TemporalStatisticsSnapshot {
  TemporalWindowStatistics short_window;
  TemporalWindowStatistics medium_window;
  TemporalWindowStatistics session;
};

struct TemporalStatisticsAggregatorConfig {
  std::int64_t short_window_ns{10'000'000'000LL};
  std::int64_t medium_window_ns{60'000'000'000LL};
};

class TemporalStatisticsAggregator final {
 public:
  explicit TemporalStatisticsAggregator(TemporalStatisticsAggregatorConfig config = TemporalStatisticsAggregatorConfig{});

  void Observe(
      std::int64_t observed_at_ns,
      const causal_slam::model::TemporalObservation& observation,
      causal_slam::telemetry::TemporalHealthStatus overall_status);

  [[nodiscard]] TemporalStatisticsSnapshot Snapshot(std::int64_t now_ns) const;

 private:
  struct Sample {
    std::int64_t observed_at_ns{0};
    causal_slam::telemetry::TemporalHealthStatus overall_status{
        causal_slam::telemetry::TemporalHealthStatus::kOk};
    causal_slam::model::TemporalObservation observation;
  };

  void PruneRollingSamples(std::int64_t now_ns);

  [[nodiscard]] TemporalWindowStatistics BuildWindowStatistics(const std::vector<Sample>& samples, std::int64_t window_duration_ns) const;

  [[nodiscard]] std::vector<Sample> SamplesSince(std::int64_t cutoff_ns) const;

  TemporalStatisticsAggregatorConfig config_;

  std::deque<Sample> rolling_samples_;
  std::vector<Sample> session_samples_;
};

}  // namespace causal_slam::statistics