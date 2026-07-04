#pragma once

#include <cstdint>
#include <optional>

#include "domain/telemetry/stream_timing_diagnostic.h"

namespace causal_slam::telemetry {

enum class StreamStalenessState {
  kMissing,
  kFresh,
  kStale,
};

struct StreamStalenessConfig {
  bool enabled{true};
  std::int64_t max_staleness_ns{500'000'000LL};
};

struct StreamStalenessSummary {
  TemporalStreamId stream_id{TemporalStreamId::kUnknown};
  StreamStalenessState state{StreamStalenessState::kMissing};
  bool enabled{true};

  std::optional<std::int64_t> latest_arrival_time_ns;
  std::int64_t now_arrival_time_ns{0};
  std::int64_t max_staleness_ns{0};
  std::int64_t age_ns{0};

  [[nodiscard]] bool IsFresh() const { return state == StreamStalenessState::kFresh; }

  [[nodiscard]] bool IsStale() const { return state == StreamStalenessState::kStale; }

  [[nodiscard]] bool IsMissing() const { return state == StreamStalenessState::kMissing; }
};

class StreamStalenessAnalyzer final {
 public:
  explicit StreamStalenessAnalyzer(TemporalStreamId stream_id, StreamStalenessConfig config = StreamStalenessConfig{});

  void SetConfig(const StreamStalenessConfig& config);

  [[nodiscard]] StreamStalenessSummary Analyze(std::optional<std::int64_t> latest_arrival_time_ns, std::int64_t now_arrival_time_ns) const;

 private:
  TemporalStreamId stream_id_{TemporalStreamId::kUnknown};
  StreamStalenessConfig config_;
};

}  // namespace causal_slam::telemetry