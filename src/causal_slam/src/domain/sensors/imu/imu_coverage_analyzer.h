#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "domain/time/time_window.h"

namespace causal_slam::coverage {

struct ImuSample {
  std::int64_t stamp_ns{0};
};

enum class ImuCoverageHealth {
  kOk,
  kWarning,
  kDegraded,
};

[[nodiscard]] const char* ToString(ImuCoverageHealth health);

struct ImuCoverageConfig {
  double max_missing_prefix_ms{5.0};
  double max_missing_suffix_ms{5.0};
  double max_internal_gap_ms{30.0};
};

struct ImuCoverageSummary {
  std::uint64_t imu_count_in_window{0};

  double missing_prefix_ms{0.0};
  double missing_suffix_ms{0.0};
  double max_gap_inside_ms{0.0};
  double coverage_ratio{0.0};

  ImuCoverageHealth health{ImuCoverageHealth::kOk};
  std::string reason{"ok"};
};

class ImuCoverageAnalyzer final {
 public:
  explicit ImuCoverageAnalyzer(ImuCoverageConfig config = ImuCoverageConfig{});

  void SetConfig(ImuCoverageConfig config);

  template <typename ImuSamplesRange>
  [[nodiscard]] ImuCoverageSummary Analyze(
      causal_slam::core::TimeWindow scan_window,
      const ImuSamplesRange& imu_samples) const {
    std::vector<std::int64_t> stamps;
    stamps.reserve(imu_samples.size());

    for (const auto& sample : imu_samples) {
      if (scan_window.Contains(sample.stamp_ns)) {
        stamps.push_back(sample.stamp_ns);
      }
    }

    return AnalyzeStamps(scan_window, std::move(stamps));
  }

 private:
  [[nodiscard]] ImuCoverageSummary AnalyzeStamps(
      causal_slam::core::TimeWindow scan_window,
      std::vector<std::int64_t> stamps) const;

  ImuCoverageConfig config_;
};

}  // namespace causal_slam::coverage