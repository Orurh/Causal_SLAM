#include "domain/telemetry/stream_staleness_analyzer.h"

#include <algorithm>

namespace causal_slam::telemetry {

StreamStalenessAnalyzer::StreamStalenessAnalyzer(TemporalStreamId stream_id, StreamStalenessConfig config)
    : stream_id_(stream_id), config_(config) {}

void StreamStalenessAnalyzer::SetConfig(const StreamStalenessConfig& config) {
  config_ = config;
}

StreamStalenessSummary StreamStalenessAnalyzer::Analyze(std::optional<std::int64_t> latest_arrival_time_ns,
                                                        std::int64_t now_arrival_time_ns) const {
  StreamStalenessSummary summary{
      .stream_id = stream_id_,
      .state = StreamStalenessState::kMissing,
      .enabled = config_.enabled,
      .latest_arrival_time_ns = latest_arrival_time_ns,
      .now_arrival_time_ns = now_arrival_time_ns,
      .max_staleness_ns = std::max<std::int64_t>(config_.max_staleness_ns, 1),
      .age_ns = 0,
  };

  if (!config_.enabled) {
    summary.state = StreamStalenessState::kFresh;
    return summary;
  }

  if (!latest_arrival_time_ns.has_value()) {
    summary.state = StreamStalenessState::kMissing;
    return summary;
  }

  summary.age_ns = std::max<std::int64_t>(now_arrival_time_ns - *latest_arrival_time_ns, 0);

  if (summary.age_ns > summary.max_staleness_ns) {
    summary.state = StreamStalenessState::kStale;
    return summary;
  }

  summary.state = StreamStalenessState::kFresh;
  return summary;
}

}  // namespace causal_slam::telemetry