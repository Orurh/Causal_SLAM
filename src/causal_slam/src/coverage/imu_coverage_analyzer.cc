#include "coverage/imu_coverage_analyzer.h"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace causal_slam::coverage {
namespace {

constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

double NanosecondsToMilliseconds(const std::int64_t nanoseconds) {
  return static_cast<double>(nanoseconds) / kNanosecondsPerMillisecond;
}

double ClampRatio(const double value) {
  return std::clamp(value, 0.0, 1.0);
}

double MaxGapInsideMilliseconds(const std::vector<std::int64_t>& stamps) {
  if (stamps.size() < 2) {
    return 0.0;
  }

  std::int64_t max_gap_ns = 0;

  for (std::size_t i = 1; i < stamps.size(); ++i) {
    max_gap_ns = std::max(max_gap_ns, stamps[i] - stamps[i - 1]);
  }

  return NanosecondsToMilliseconds(max_gap_ns);
}

ImuCoverageSummary EvaluateCoverageHealth(
    ImuCoverageSummary summary,
    const ImuCoverageConfig& config) {
  if (summary.imu_count_in_window == 0) {
    summary.health = ImuCoverageHealth::kDegraded;
    summary.reason = "imu_window_empty";
    return summary;
  }

  if (summary.missing_prefix_ms > config.max_missing_prefix_ms) {
    summary.health = ImuCoverageHealth::kDegraded;
    summary.reason = "imu_window_missing_prefix";
    return summary;
  }

  if (summary.missing_suffix_ms > config.max_missing_suffix_ms) {
    summary.health = ImuCoverageHealth::kDegraded;
    summary.reason = "imu_window_missing_suffix";
    return summary;
  }

  if (summary.max_gap_inside_ms > config.max_internal_gap_ms) {
    summary.health = ImuCoverageHealth::kDegraded;
    summary.reason = "imu_window_internal_gap";
    return summary;
  }

  summary.health = ImuCoverageHealth::kOk;
  summary.reason = "ok";
  return summary;
}

}  // namespace

const char* ToString(const ImuCoverageHealth health) {
  switch (health) {
    case ImuCoverageHealth::kOk:
      return "OK";
    case ImuCoverageHealth::kWarning:
      return "WARNING";
    case ImuCoverageHealth::kDegraded:
      return "DEGRADED";
  }

  return "UNKNOWN";
}

ImuCoverageAnalyzer::ImuCoverageAnalyzer(ImuCoverageConfig config)
    : config_(config) {}

void ImuCoverageAnalyzer::SetConfig(ImuCoverageConfig config) {
  config_ = config;
}

ImuCoverageSummary ImuCoverageAnalyzer::AnalyzeStamps(
    const causal_slam::core::TimeWindow scan_window,
    std::vector<std::int64_t> stamps) const {
  if (!scan_window.IsValid() || scan_window.DurationNs() <= 0) {
    return ImuCoverageSummary{
        .health = ImuCoverageHealth::kDegraded,
        .reason = "invalid_scan_window",
    };
  }

  std::ranges::sort(stamps);

  const double scan_duration_ms =
      NanosecondsToMilliseconds(scan_window.DurationNs());

  if (stamps.empty()) {
    return ImuCoverageSummary{
        .imu_count_in_window = 0,
        .missing_prefix_ms = scan_duration_ms,
        .missing_suffix_ms = scan_duration_ms,
        .max_gap_inside_ms = 0.0,
        .coverage_ratio = 0.0,
        .health = ImuCoverageHealth::kDegraded,
        .reason = "imu_window_empty",
    };
  }

  const std::int64_t first_imu_ns = stamps.front();
  const std::int64_t last_imu_ns = stamps.back();

  const double missing_prefix_ms =
      NanosecondsToMilliseconds(first_imu_ns - scan_window.start_ns);
  const double missing_suffix_ms =
      NanosecondsToMilliseconds(scan_window.end_ns - last_imu_ns);
  const double max_gap_inside_ms = MaxGapInsideMilliseconds(stamps);

  const double covered_ms =
      scan_duration_ms - missing_prefix_ms - missing_suffix_ms;
  const double coverage_ratio =
      scan_duration_ms > 0.0 ? ClampRatio(covered_ms / scan_duration_ms) : 0.0;

  auto summary = ImuCoverageSummary{
      .imu_count_in_window = static_cast<std::uint64_t>(stamps.size()),
      .missing_prefix_ms = missing_prefix_ms,
      .missing_suffix_ms = missing_suffix_ms,
      .max_gap_inside_ms = max_gap_inside_ms,
      .coverage_ratio = coverage_ratio,
  };

  return EvaluateCoverageHealth(std::move(summary), config_);
}

}  // namespace causal_slam::coverage