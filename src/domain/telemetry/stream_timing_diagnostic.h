#pragma once

#include <utility>

#include "stream_timing_tracker.h"
#include "temporal_stream.h"

namespace causal_slam::telemetry {

struct StreamTimingDiagnostic {
  TemporalStreamId id{TemporalStreamId::kUnknown};
  TimingSummary timing;
};

[[nodiscard]] inline StreamTimingDiagnostic MakeStreamTimingDiagnostic(
    TemporalStreamId id,
    TimingSummary timing) {
  return StreamTimingDiagnostic{
      .id = id,
      .timing = std::move(timing),
  };
}

}  // namespace causal_slam::telemetry
