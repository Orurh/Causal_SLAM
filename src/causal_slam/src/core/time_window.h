#pragma once

#include <cstdint>

namespace causal_slam::core {

struct TimeWindow {
  std::int64_t start_ns{0};
  std::int64_t end_ns{0};

  [[nodiscard]] bool IsValid() const {
    return end_ns >= start_ns;
  }

  [[nodiscard]] std::int64_t DurationNs() const {
    return IsValid() ? end_ns - start_ns : 0;
  }

  [[nodiscard]] bool Contains(const std::int64_t stamp_ns) const {
    return IsValid() && stamp_ns >= start_ns && stamp_ns <= end_ns;
  }
};

}  // namespace causal_slam::core