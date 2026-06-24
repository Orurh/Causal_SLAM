#include "lidar/lidar_scan_window_estimator.h"

#include "domain/time/time_units.h"

#include <algorithm>
#include <cstdint>
#include <utility>

namespace causal_slam::lidar {
namespace {

double SanitizePositiveDurationMs(double duration_ms, double fallback_ms) {
  if (duration_ms > 0.0) {
    return duration_ms;
  }

  return std::max(fallback_ms, 1.0);
}

}  // namespace

const char* ToString(LidarScanWindowSource source) {
  switch (source) {
    case LidarScanWindowSource::kAssumedFixedDuration:
      return "assumed_fixed_duration";
    case LidarScanWindowSource::kMeasuredHeaderPeriod:
      return "measured_header_period";
    case LidarScanWindowSource::kPointTimeField:
      return "point_time_field";
    case LidarScanWindowSource::kDriverMetadata:
      return "driver_metadata";
  }

  return "unknown";
}

const char* ToString(LidarScanWindowConfidence confidence) {
  switch (confidence) {
    case LidarScanWindowConfidence::kLow:
      return "LOW";
    case LidarScanWindowConfidence::kMedium:
      return "MEDIUM";
    case LidarScanWindowConfidence::kHigh:
      return "HIGH";
  }

  return "UNKNOWN";
}

LidarScanWindowEstimator::LidarScanWindowEstimator(
    LidarScanWindowEstimatorConfig config)
    : config_(config) {}

void LidarScanWindowEstimator::SetConfig(
    LidarScanWindowEstimatorConfig config) {
  config_ = config;
  previous_header_stamp_ns_.reset();
}

LidarScanWindowEstimate LidarScanWindowEstimator::Estimate(
    std::int64_t header_stamp_ns) {
  if (!config_.prefer_measured_header_period) {
    previous_header_stamp_ns_ = header_stamp_ns;
    return BuildFallbackEstimate(
        header_stamp_ns, "measured_header_period_disabled");
  }

  if (!previous_header_stamp_ns_.has_value()) {
    previous_header_stamp_ns_ = header_stamp_ns;
    return BuildFallbackEstimate(header_stamp_ns, "no_previous_lidar_stamp");
  }

  const std::int64_t measured_duration_ns =
      header_stamp_ns - *previous_header_stamp_ns_;
  previous_header_stamp_ns_ = header_stamp_ns;

  if (measured_duration_ns <= 0) {
    return BuildFallbackEstimate(header_stamp_ns,
                                 "invalid_measured_header_period");
  }

  const double measured_duration_ms =
      causal_slam::core::NanosecondsToMilliseconds(measured_duration_ns);

  if (measured_duration_ms < config_.min_measured_scan_duration_ms ||
      measured_duration_ms > config_.max_measured_scan_duration_ms) {
    return BuildFallbackEstimate(header_stamp_ns,
                                 "measured_header_period_out_of_range");
  }

  return LidarScanWindowEstimate{
      .window = BuildLidarScanWindow(
          header_stamp_ns, measured_duration_ms, config_.stamp_policy),
      .duration_ms = measured_duration_ms,
      .source = LidarScanWindowSource::kMeasuredHeaderPeriod,
      .confidence = LidarScanWindowConfidence::kMedium,
      .reason = "measured_header_period",
  };
}

LidarScanWindowEstimate LidarScanWindowEstimator::BuildFallbackEstimate(
    std::int64_t header_stamp_ns,
    std::string reason) const {
  const double fallback_duration_ms =
      SanitizePositiveDurationMs(config_.fallback_scan_duration_ms, 100.0);

  return LidarScanWindowEstimate{
      .window = BuildLidarScanWindow(
          header_stamp_ns, fallback_duration_ms, config_.stamp_policy),
      .duration_ms = fallback_duration_ms,
      .source = LidarScanWindowSource::kAssumedFixedDuration,
      .confidence = LidarScanWindowConfidence::kLow,
      .reason = std::move(reason),
  };
}

}  // namespace causal_slam::lidar
