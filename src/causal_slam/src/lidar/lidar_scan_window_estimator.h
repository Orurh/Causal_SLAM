#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "core/time_window.h"
#include "lidar/lidar_scan_timing.h"

namespace causal_slam::lidar {

enum class LidarScanWindowSource : std::uint8_t {
  kAssumedFixedDuration,
  kMeasuredHeaderPeriod,
  kPointTimeField,
  kDriverMetadata,
};

enum class LidarScanWindowConfidence : std::uint8_t {
  kLow,
  kMedium,
  kHigh,
};

[[nodiscard]] const char* ToString(LidarScanWindowSource source);
[[nodiscard]] const char* ToString(LidarScanWindowConfidence confidence);

struct LidarScanWindowEstimatorConfig {
  double fallback_scan_duration_ms{100.0};
  double min_measured_scan_duration_ms{1.0};
  double max_measured_scan_duration_ms{500.0};

  LidarStampPolicy stamp_policy{LidarStampPolicy::kScanEnd};
  bool prefer_measured_header_period{true};
};

struct LidarScanWindowEstimate {
  causal_slam::core::TimeWindow window;

  double duration_ms{0.0};
  LidarScanWindowSource source{LidarScanWindowSource::kAssumedFixedDuration};
  LidarScanWindowConfidence confidence{LidarScanWindowConfidence::kLow};

  std::string reason{"assumed_fixed_duration"};
};

class LidarScanWindowEstimator final {
 public:
  explicit LidarScanWindowEstimator(LidarScanWindowEstimatorConfig config = LidarScanWindowEstimatorConfig{});

  void SetConfig(LidarScanWindowEstimatorConfig config);

  [[nodiscard]] LidarScanWindowEstimate Estimate(std::int64_t header_stamp_ns);

 private:
  [[nodiscard]] LidarScanWindowEstimate BuildFallbackEstimate(std::int64_t header_stamp_ns, std::string reason) const;

  LidarScanWindowEstimatorConfig config_;
  std::optional<std::int64_t> previous_header_stamp_ns_;
};

}  // namespace causal_slam::lidar