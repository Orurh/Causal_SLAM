#include "lidar/lidar_scan_timing.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace causal_slam::lidar {
namespace {

constexpr auto kNanosecondsPerMillisecond = 1'000'000.0;

std::int64_t MillisecondsToNanoseconds(const double milliseconds) {
  const double safe_milliseconds = std::max(milliseconds, 0.0);
  return static_cast<std::int64_t>(std::llround(safe_milliseconds * kNanosecondsPerMillisecond));
}

}  // namespace

const char* ToString(const LidarStampPolicy policy) {
  switch (policy) {
    case LidarStampPolicy::kScanStart:
      return "scan_start";
    case LidarStampPolicy::kScanMiddle:
      return "scan_middle";
    case LidarStampPolicy::kScanEnd:
      return "scan_end";
  }

  return "unknown";
}

causal_slam::core::TimeWindow BuildLidarScanWindow(const std::int64_t header_stamp_ns, const double scan_duration_ms,
                                                   const LidarStampPolicy stamp_policy) {
  const std::int64_t duration_ns = MillisecondsToNanoseconds(scan_duration_ms);

  switch (stamp_policy) {
    case LidarStampPolicy::kScanStart:
      return causal_slam::core::TimeWindow{
          .start_ns = header_stamp_ns,
          .end_ns = header_stamp_ns + duration_ns,
      };

    case LidarStampPolicy::kScanMiddle: {
      const std::int64_t left_duration_ns = duration_ns / 2;
      const std::int64_t right_duration_ns = duration_ns - left_duration_ns;

      return causal_slam::core::TimeWindow{
          .start_ns = header_stamp_ns - left_duration_ns,
          .end_ns = header_stamp_ns + right_duration_ns,
      };
    }

    case LidarStampPolicy::kScanEnd:
      return causal_slam::core::TimeWindow{
          .start_ns = header_stamp_ns - duration_ns,
          .end_ns = header_stamp_ns,
      };
  }

  return causal_slam::core::TimeWindow{
      .start_ns = header_stamp_ns,
      .end_ns = header_stamp_ns,
  };
}

}  // namespace causal_slam::lidar