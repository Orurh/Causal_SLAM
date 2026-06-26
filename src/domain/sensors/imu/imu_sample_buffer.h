#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "imu_coverage_analyzer.h"

namespace causal_slam::coverage {

class ImuSampleBuffer final {
 public:
  explicit ImuSampleBuffer(std::int64_t retention_ns);

  void Add(ImuSample sample);
  void PruneOlderThan(std::int64_t cutoff_ns);

  [[nodiscard]] const std::deque<ImuSample>& Samples() const;
  [[nodiscard]] std::size_t Size() const;
  [[nodiscard]] std::uint64_t DroppedOutOfOrderCount() const;

 private:
  std::int64_t retention_ns_{0};
  std::uint64_t dropped_out_of_order_count_{0};
  std::deque<ImuSample> samples_;
};

}  // namespace causal_slam::coverage