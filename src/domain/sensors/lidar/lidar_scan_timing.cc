#include "lidar_scan_timing.h"

#include "domain/time/time_units.h"

#include <algorithm>
#include <cstdint>

namespace causal_slam::lidar {

const char* ToString(const LidarStampPolicy stamp_policy) {
  switch (stamp_policy) {
    case LidarStampPolicy::kScanStart:
      return "scan_start";
    case LidarStampPolicy::kScanMiddle:
      return "scan_middle";
    case LidarStampPolicy::kScanEnd:
      return "scan_end";
  }

  return "unknown";
}

causal_slam::core::TimeWindow BuildLidarScanWindow(const std::int64_t stamp_ns, const double scan_duration_ms,
                                                   const LidarStampPolicy stamp_policy) {
  const std::int64_t duration_ns = causal_slam::core::MillisecondsToNanoseconds(std::max(scan_duration_ms, 0.0));

  switch (stamp_policy) {
    case LidarStampPolicy::kScanStart:
      return causal_slam::core::TimeWindow{stamp_ns, stamp_ns + duration_ns};

    case LidarStampPolicy::kScanMiddle:
      return causal_slam::core::TimeWindow{stamp_ns - (duration_ns / 2), stamp_ns + (duration_ns / 2)};

    case LidarStampPolicy::kScanEnd:
      return causal_slam::core::TimeWindow{stamp_ns - duration_ns, stamp_ns};
  }

  return causal_slam::core::TimeWindow{stamp_ns - duration_ns, stamp_ns};
}

}  // namespace causal_slam::lidar