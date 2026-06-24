#pragma once

#include <cstdint>
#include <string>

namespace causal_slam::transform {

struct TransformLookupObservation {
  std::string target_frame;
  std::string source_frame;

  // Timestamp for which the transform was requested.
  std::int64_t requested_stamp_ns{0};

  // Timestamp of the transform that was actually used.
  std::int64_t transform_stamp_ns{0};

  // Local receive/check time of the sensor message or lookup attempt.
  std::int64_t receive_time_ns{0};

  bool lookup_success{false};
  bool extrapolation_required{false};

  // Adapter-provided failure detail, for example:
  // "lookup_failed", "extrapolation_into_future", "frame_not_found".
  std::string failure_reason;
};

}  // namespace causal_slam::transform
