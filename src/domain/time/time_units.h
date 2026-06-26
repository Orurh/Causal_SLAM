#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace causal_slam::core {

inline constexpr double kNanosecondsPerMillisecond = 1'000'000.0;
inline constexpr double kNanosecondsPerSecond = 1'000'000'000.0;

[[nodiscard]] inline double NanosecondsToMilliseconds(std::int64_t nanoseconds) {
  return static_cast<double>(nanoseconds) / kNanosecondsPerMillisecond;
}

[[nodiscard]] inline double NanosecondsToSeconds(std::int64_t nanoseconds) {
  return static_cast<double>(nanoseconds) / kNanosecondsPerSecond;
}

[[nodiscard]] inline std::int64_t SaturatingRoundToInt64(double value) {
  if (!std::isfinite(value)) {
    return 0;
  }

  if (value >= static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return std::numeric_limits<std::int64_t>::max();
  }

  if (value <= static_cast<double>(std::numeric_limits<std::int64_t>::min())) {
    return std::numeric_limits<std::int64_t>::min();
  }

  return static_cast<std::int64_t>(std::llround(value));
}

[[nodiscard]] inline std::int64_t MillisecondsToNanoseconds(double milliseconds) {
  return SaturatingRoundToInt64(milliseconds * kNanosecondsPerMillisecond);
}

[[nodiscard]] inline std::int64_t SecondsToNanoseconds(double seconds) {
  return SaturatingRoundToInt64(seconds * kNanosecondsPerSecond);
}

}  // namespace causal_slam::core
