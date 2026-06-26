#include "temporal_statistics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace causal_slam::statistics {
namespace {

struct StreamTimingAccumulator {
  causal_slam::telemetry::TemporalStreamId id{
      causal_slam::telemetry::TemporalStreamId::kUnknown};

  std::vector<double> delay_ms;
  std::vector<double> period_ms;
  std::vector<double> jitter_ms;
};

bool IsFinite(double value) {
  return std::isfinite(value);
}

NumericStats BuildNumericStats(std::vector<double> values) {
  values.erase(std::remove_if(values.begin(), values.end(), [](double value) { return !IsFinite(value); }), values.end());

  NumericStats stats;
  stats.count = static_cast<std::uint64_t>(values.size());

  if (values.empty()) {
    return stats;
  }

  std::sort(values.begin(), values.end());

  stats.min = values.front();
  stats.max = values.back();

  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }

  stats.mean = sum / static_cast<double>(values.size());

  const std::size_t size = values.size();
  const std::size_t middle = size / 2;

  if (size % 2 == 0) {
    stats.median = (values[middle - 1] + values[middle]) / 2.0;
  } else {
    stats.median = values[middle];
  }

  const std::size_t p95_index = static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(size))) - 1;
  stats.p95 = values[std::min(p95_index, size - 1)];

  return stats;
}

void AddHealthSample(causal_slam::telemetry::TemporalHealthStatus status, HealthDistribution* distribution) {
  switch (status) {
    case causal_slam::telemetry::TemporalHealthStatus::kOk:
      ++distribution->ok_count;
      break;
    case causal_slam::telemetry::TemporalHealthStatus::kWarning:
      ++distribution->warning_count;
      break;
    case causal_slam::telemetry::TemporalHealthStatus::kDegraded:
      ++distribution->degraded_count;
      break;
    case causal_slam::telemetry::TemporalHealthStatus::kInvalid:
      ++distribution->invalid_count;
      break;
  }
}

void AddScanWindowSourceSample(causal_slam::lidar::LidarScanWindowSource source, LidarScanWindowSourceDistribution* distribution) {
  switch (source) {
    case causal_slam::lidar::LidarScanWindowSource::kAssumedFixedDuration:
      ++distribution->assumed_fixed_duration_count;
      break;
    case causal_slam::lidar::LidarScanWindowSource::kMeasuredHeaderPeriod:
      ++distribution->measured_header_period_count;
      break;
    case causal_slam::lidar::LidarScanWindowSource::kPointTimeField:
      ++distribution->point_time_field_count;
      break;
    case causal_slam::lidar::LidarScanWindowSource::kDriverMetadata:
      ++distribution->driver_metadata_count;
      break;
  }
}

void AddScanWindowConfidenceSample(causal_slam::lidar::LidarScanWindowConfidence confidence,
                                   LidarScanWindowConfidenceDistribution* distribution) {
  switch (confidence) {
    case causal_slam::lidar::LidarScanWindowConfidence::kLow:
      ++distribution->low_count;
      break;
    case causal_slam::lidar::LidarScanWindowConfidence::kMedium:
      ++distribution->medium_count;
      break;
    case causal_slam::lidar::LidarScanWindowConfidence::kHigh:
      ++distribution->high_count;
      break;
  }
}

StreamTimingAccumulator& FindOrAppendStreamAccumulator(
    std::vector<StreamTimingAccumulator>* accumulators,
    causal_slam::telemetry::TemporalStreamId id) {
  auto it = std::find_if(
      accumulators->begin(), accumulators->end(), [id](const auto& item) {
        return item.id == id;
      });

  if (it != accumulators->end()) {
    return *it;
  }

  accumulators->push_back(StreamTimingAccumulator{
      .id = id,
      .delay_ms = {},
      .period_ms = {},
      .jitter_ms = {},
  });

  return accumulators->back();
}

StreamTimingStatistics BuildStreamTimingStatistics(StreamTimingAccumulator accumulator) {
  return StreamTimingStatistics{
      .id = accumulator.id,
      .delay_ms = BuildNumericStats(std::move(accumulator.delay_ms)),
      .period_ms = BuildNumericStats(std::move(accumulator.period_ms)),
      .jitter_ms = BuildNumericStats(std::move(accumulator.jitter_ms)),
  };
}

void IncrementBlockReason(
    const std::string& reason,
    std::vector<CloudBlockReasonCount>* counts) {
  const std::string key = reason.empty() ? "unknown" : reason;

  auto it = std::find_if(counts->begin(), counts->end(), [&](const auto& item) {
    return item.reason == key;
  });

  if (it != counts->end()) {
    ++it->count;
    return;
  }

  counts->push_back(CloudBlockReasonCount{
      .reason = key,
      .count = 1,
  });
}

}  // namespace

const char* ToString(CloudForwardingDecision decision) {
  switch (decision) {
    case CloudForwardingDecision::kForwarded:
      return "forwarded";
    case CloudForwardingDecision::kBlockedWarmup:
      return "blocked_warmup";
    case CloudForwardingDecision::kBlockedByGate:
      return "blocked_by_gate";
  }

  return "unknown";
}

TemporalStatisticsAggregator::TemporalStatisticsAggregator(TemporalStatisticsAggregatorConfig config) : config_(config) {}

void TemporalStatisticsAggregator::Observe(
    std::int64_t observed_at_ns,
    const causal_slam::model::TemporalObservation& observation,
    causal_slam::telemetry::TemporalHealthStatus overall_status) {
  Sample sample{
      .observed_at_ns = observed_at_ns,
      .overall_status = overall_status,
      .observation = observation,
  };

  rolling_samples_.push_back(sample);
  session_samples_.push_back(sample);

  PruneRollingSamples(observed_at_ns);
}

void TemporalStatisticsAggregator::ObserveCloudDecision(
    const CloudDecisionEvent& event) {
  ++cloud_decision_total_count_;

  switch (event.decision) {
    case CloudForwardingDecision::kForwarded:
      ++cloud_decision_forwarded_count_;
      return;

    case CloudForwardingDecision::kBlockedWarmup:
      ++cloud_decision_blocked_warmup_count_;
      break;

    case CloudForwardingDecision::kBlockedByGate:
      ++cloud_decision_blocked_by_gate_count_;
      break;
  }

  IncrementBlockReason(event.reason, &cloud_block_reasons_);
  recent_blocked_cloud_events_.push_back(event);

  const std::uint64_t limit =
      std::max<std::uint64_t>(config_.max_recent_blocked_cloud_events, 1);

  while (recent_blocked_cloud_events_.size() > limit) {
    recent_blocked_cloud_events_.pop_front();
  }
}

TemporalStatisticsSnapshot TemporalStatisticsAggregator::Snapshot(std::int64_t now_ns) const {
  const std::int64_t short_cutoff_ns = now_ns - std::max<std::int64_t>(config_.short_window_ns, 0);
  const std::int64_t medium_cutoff_ns = now_ns - std::max<std::int64_t>(config_.medium_window_ns, 0);

  TemporalStatisticsSnapshot snapshot;
  snapshot.short_window = BuildWindowStatistics(SamplesSince(short_cutoff_ns), config_.short_window_ns);
  snapshot.medium_window = BuildWindowStatistics(SamplesSince(medium_cutoff_ns), config_.medium_window_ns);
  snapshot.session = BuildWindowStatistics(session_samples_, 0);
  snapshot.cloud_decisions = BuildCloudDecisionStatistics();

  return snapshot;
}

void TemporalStatisticsAggregator::PruneRollingSamples(std::int64_t now_ns) {
  const std::int64_t max_window_ns = std::max<std::int64_t>(config_.medium_window_ns, config_.short_window_ns);
  const std::int64_t cutoff_ns = now_ns - std::max<std::int64_t>(max_window_ns, 0);

  while (!rolling_samples_.empty() && rolling_samples_.front().observed_at_ns < cutoff_ns) {
    rolling_samples_.pop_front();
  }
}

std::vector<TemporalStatisticsAggregator::Sample> TemporalStatisticsAggregator::SamplesSince(std::int64_t cutoff_ns) const {
  std::vector<Sample> result;

  for (const auto& sample : rolling_samples_) {
    if (sample.observed_at_ns >= cutoff_ns) {
      result.push_back(sample);
    }
  }

  return result;
}

TemporalWindowStatistics TemporalStatisticsAggregator::BuildWindowStatistics(const std::vector<Sample>& samples,
                                                                             std::int64_t window_duration_ns) const {
  TemporalWindowStatistics stats;
  stats.window_duration_ns = window_duration_ns;
  stats.sample_count = static_cast<std::uint64_t>(samples.size());

  std::vector<StreamTimingAccumulator> stream_accumulators;

  std::vector<double> imu_coverage_ratio;
  std::vector<double> imu_samples_in_window;
  std::vector<double> imu_max_gap_inside_ms;

  imu_coverage_ratio.reserve(samples.size());
  imu_samples_in_window.reserve(samples.size());
  imu_max_gap_inside_ms.reserve(samples.size());

  for (const auto& sample : samples) {
    AddHealthSample(sample.overall_status, &stats.health);

    if (sample.observation.lidar_scan_window.has_value()) {
      AddScanWindowSourceSample(sample.observation.lidar_scan_window->source,
                                &stats.scan_window_sources);
      AddScanWindowConfidenceSample(sample.observation.lidar_scan_window->confidence,
                                    &stats.scan_window_confidence);
    }

    for (const auto& stream : sample.observation.streams) {
      const auto& timing = stream.timing;

      if (timing.total_count == 0) {
        continue;
      }

      auto& accumulator =
          FindOrAppendStreamAccumulator(&stream_accumulators, stream.id);
      accumulator.delay_ms.push_back(timing.last_delay_ms);
      accumulator.period_ms.push_back(timing.last_period_ms);
      accumulator.jitter_ms.push_back(timing.last_jitter_ms);
    }

    if (sample.observation.imu_coverage.has_value()) {
      imu_coverage_ratio.push_back(sample.observation.imu_coverage->coverage_ratio);
      imu_samples_in_window.push_back(
          static_cast<double>(sample.observation.imu_coverage->imu_count_in_window));
      imu_max_gap_inside_ms.push_back(sample.observation.imu_coverage->max_gap_inside_ms);
    }
  }

  stats.streams.reserve(stream_accumulators.size());
  for (auto& accumulator : stream_accumulators) {
    stats.streams.push_back(BuildStreamTimingStatistics(std::move(accumulator)));
  }

  stats.imu_coverage_ratio = BuildNumericStats(std::move(imu_coverage_ratio));
  stats.imu_samples_in_window = BuildNumericStats(std::move(imu_samples_in_window));
  stats.imu_max_gap_inside_ms = BuildNumericStats(std::move(imu_max_gap_inside_ms));

  return stats;
}

}  // namespace causal_slam::statistics
namespace causal_slam::statistics {

CloudDecisionStatistics
TemporalStatisticsAggregator::BuildCloudDecisionStatistics() const {
  CloudDecisionStatistics stats;
  stats.total_count = cloud_decision_total_count_;
  stats.forwarded_count = cloud_decision_forwarded_count_;
  stats.blocked_warmup_count = cloud_decision_blocked_warmup_count_;
  stats.blocked_by_gate_count = cloud_decision_blocked_by_gate_count_;
  stats.blocked_count =
      stats.blocked_warmup_count + stats.blocked_by_gate_count;

  stats.block_reasons = cloud_block_reasons_;
  std::sort(
      stats.block_reasons.begin(),
      stats.block_reasons.end(),
      [](const auto& lhs, const auto& rhs) {
        if (lhs.count != rhs.count) {
          return lhs.count > rhs.count;
        }
        return lhs.reason < rhs.reason;
      });

  stats.recent_blocked_events.assign(
      recent_blocked_cloud_events_.begin(),
      recent_blocked_cloud_events_.end());

  return stats;
}

}  // namespace causal_slam::statistics
