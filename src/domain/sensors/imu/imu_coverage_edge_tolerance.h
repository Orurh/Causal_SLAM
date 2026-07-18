#pragma once

#include <cstdint>
#include <optional>

namespace causal_slam::coverage {

enum class ImuCoverageEdgeToleranceSource : std::uint8_t {
  kConfiguredMinimum,
  kObservedPeriodP95,
};

[[nodiscard]] const char* ToString(ImuCoverageEdgeToleranceSource source);

struct ImuCoverageEdgeToleranceConfig {
  double configured_min_tolerance_ms{5.0};
  double observed_period_multiplier{1.5};
};

struct ImuCoverageEdgeToleranceResolution {
  double effective_tolerance_ms{0.0};
  double configured_min_tolerance_ms{0.0};
  double observed_period_p95_ms{0.0};
  double adaptive_tolerance_ms{0.0};

  ImuCoverageEdgeToleranceSource source{ImuCoverageEdgeToleranceSource::kConfiguredMinimum};
};

[[nodiscard]] ImuCoverageEdgeToleranceResolution ResolveImuCoverageEdgeTolerance(const ImuCoverageEdgeToleranceConfig& config,
                                                                                 std::optional<double> observed_period_p95_ms);

}  // namespace causal_slam::coverage
