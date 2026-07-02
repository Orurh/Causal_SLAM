#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "domain/model/temporal_observation.h"
#include "domain/telemetry/temporal_health.h"

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
  std::uint64_t invalid_count{0};
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

enum class CloudForwardingDecision : std::uint8_t {
  kForwarded,
  kBlockedWarmup,
  kBlockedByGate,
};

[[nodiscard]] const char* ToString(CloudForwardingDecision decision);

struct CloudBlockReasonCount {
  std::string reason;
  std::uint64_t count{0};
};

struct CloudDecisionEvent {
  std::uint64_t sequence_id{0};

  std::int64_t header_stamp_ns{0};
  std::int64_t receive_time_ns{0};
  std::string frame_id;

  std::uint64_t point_count{0};
  std::uint64_t data_size_bytes{0};

  causal_slam::telemetry::TemporalHealthStatus health{causal_slam::telemetry::TemporalHealthStatus::kOk};

  bool map_update_allowed{true};
  CloudForwardingDecision decision{CloudForwardingDecision::kForwarded};

  std::string reason;
  std::vector<std::string> fault_reasons;

  bool has_scan_window{false};
  double scan_duration_ms{0.0};
  std::string scan_window_source;
  std::string scan_window_confidence;

  bool has_imu_coverage{false};
  std::uint64_t imu_samples_in_window{0};
  double imu_coverage_ratio{0.0};
  double imu_max_gap_inside_ms{0.0};
};

struct CloudDecisionStatistics {
  std::uint64_t total_count{0};
  std::uint64_t forwarded_count{0};
  std::uint64_t blocked_count{0};
  std::uint64_t blocked_warmup_count{0};
  std::uint64_t blocked_by_gate_count{0};

  std::vector<CloudBlockReasonCount> block_reasons;
  std::vector<CloudDecisionEvent> recent_blocked_events;
};

struct StreamTimingStatistics {
  causal_slam::telemetry::TemporalStreamId id{causal_slam::telemetry::TemporalStreamId::kUnknown};

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

  CloudDecisionStatistics cloud_decisions;
};

struct TemporalStatisticsAggregatorConfig {
  std::int64_t short_window_ns{10'000'000'000LL};
  std::int64_t medium_window_ns{60'000'000'000LL};

  std::uint64_t max_recent_blocked_cloud_events{50};
};

class TemporalStatisticsAggregator final {
 public:
  explicit TemporalStatisticsAggregator(TemporalStatisticsAggregatorConfig config = TemporalStatisticsAggregatorConfig{});

  void Observe(std::int64_t observed_at_ns, const causal_slam::model::TemporalObservation& observation,
               causal_slam::telemetry::TemporalHealthStatus overall_status);

  void ObserveCloudDecision(const CloudDecisionEvent& event);

  [[nodiscard]] TemporalStatisticsSnapshot Snapshot(std::int64_t now_ns) const;

 private:
  struct Sample {
    std::int64_t observed_at_ns{0};
    causal_slam::telemetry::TemporalHealthStatus overall_status{causal_slam::telemetry::TemporalHealthStatus::kOk};
    causal_slam::model::TemporalObservation observation;
  };

  void PruneRollingSamples(std::int64_t now_ns);

  [[nodiscard]] TemporalWindowStatistics BuildWindowStatistics(const std::vector<Sample>& samples, std::int64_t window_duration_ns) const;

  [[nodiscard]] std::vector<Sample> SamplesSince(std::int64_t cutoff_ns) const;

  [[nodiscard]] CloudDecisionStatistics BuildCloudDecisionStatistics() const;

  TemporalStatisticsAggregatorConfig config_;

  std::deque<Sample> rolling_samples_;
  std::vector<Sample> session_samples_;

  std::uint64_t cloud_decision_total_count_{0};
  std::uint64_t cloud_decision_forwarded_count_{0};
  std::uint64_t cloud_decision_blocked_warmup_count_{0};
  std::uint64_t cloud_decision_blocked_by_gate_count_{0};

  std::vector<CloudBlockReasonCount> cloud_block_reasons_;
  std::deque<CloudDecisionEvent> recent_blocked_cloud_events_;
};

}  // namespace causal_slam::statistics