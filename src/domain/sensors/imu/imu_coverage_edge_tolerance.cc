#include "domain/sensors/imu/imu_coverage_edge_tolerance.h"

#include <algorithm>
#include <cmath>

namespace causal_slam::coverage {
namespace {

double NonNegativeFiniteOrZero(double value) {
  if (!std::isfinite(value) || value < 0.0) {
    return 0.0;
  }

  return value;
}

}  // namespace

const char* ToString(ImuCoverageEdgeToleranceSource source) {
  switch (source) {
    case ImuCoverageEdgeToleranceSource::kConfiguredMinimum:
      return "configured_minimum";
    case ImuCoverageEdgeToleranceSource::kObservedPeriodP95:
      return "observed_period_p95";
  }

  return "configured_minimum";
}

ImuCoverageEdgeToleranceResolution ResolveImuCoverageEdgeTolerance(const ImuCoverageEdgeToleranceConfig& config,
                                                                   std::optional<double> observed_period_p95_ms) {
  ImuCoverageEdgeToleranceResolution result;

  result.configured_min_tolerance_ms = NonNegativeFiniteOrZero(config.configured_min_tolerance_ms);

  const double multiplier = NonNegativeFiniteOrZero(config.observed_period_multiplier);

  if (observed_period_p95_ms.has_value() && std::isfinite(*observed_period_p95_ms) && *observed_period_p95_ms > 0.0) {
    result.observed_period_p95_ms = *observed_period_p95_ms;

    const double adaptive_tolerance_ms = result.observed_period_p95_ms * multiplier;

    if (std::isfinite(adaptive_tolerance_ms)) {
      result.adaptive_tolerance_ms = NonNegativeFiniteOrZero(adaptive_tolerance_ms);
    }
  }

  result.effective_tolerance_ms = std::max(result.configured_min_tolerance_ms, result.adaptive_tolerance_ms);

  if (result.adaptive_tolerance_ms > result.configured_min_tolerance_ms) {
    result.source = ImuCoverageEdgeToleranceSource::kObservedPeriodP95;
  }

  return result;
}

}  // namespace causal_slam::coverage
