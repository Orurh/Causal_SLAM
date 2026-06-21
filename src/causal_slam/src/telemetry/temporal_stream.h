#pragma once

#include <cstdint>

namespace causal_slam::telemetry {

enum class TemporalStreamId : std::uint8_t {
  kUnknown,
  kImu,
  kLidar,
  kCamera,
  kRadar,
  kGnss,
  kTf,
};

[[nodiscard]] inline const char* ToString(TemporalStreamId id) {
  switch (id) {
    case TemporalStreamId::kUnknown:
      return "UNKNOWN";
    case TemporalStreamId::kImu:
      return "IMU";
    case TemporalStreamId::kLidar:
      return "LiDAR";
    case TemporalStreamId::kCamera:
      return "Camera";
    case TemporalStreamId::kRadar:
      return "Radar";
    case TemporalStreamId::kGnss:
      return "GNSS";
    case TemporalStreamId::kTf:
      return "TF";
  }

  return "UNKNOWN";
}

}  // namespace causal_slam::telemetry
