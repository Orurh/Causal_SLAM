#pragma once

#include <cstdint>

#include "domain/time/time_window.h"

namespace causal_slam::lidar {

enum class LidarStampPolicy : std::uint8_t {
  kScanStart,
  kScanMiddle,
  kScanEnd,
};

[[nodiscard]] const char* ToString(LidarStampPolicy policy);

[[nodiscard]] causal_slam::core::TimeWindow BuildLidarScanWindow(std::int64_t header_stamp_ns, double scan_duration_ms,
                                                                 LidarStampPolicy stamp_policy);

}  // namespace causal_slam::lidar