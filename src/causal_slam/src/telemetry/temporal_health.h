#pragma once

#include <cstdint>

namespace causal_slam::telemetry {

enum class TemporalHealthStatus : std::uint8_t {
  kOk,
  kWarning,
  kDegraded,
};

[[nodiscard]] inline const char* ToString(TemporalHealthStatus status) {
  switch (status) {
    case TemporalHealthStatus::kOk:
      return "OK";
    case TemporalHealthStatus::kWarning:
      return "WARNING";
    case TemporalHealthStatus::kDegraded:
      return "DEGRADED";
  }

  return "UNKNOWN";
}

}  // namespace causal_slam::telemetry
