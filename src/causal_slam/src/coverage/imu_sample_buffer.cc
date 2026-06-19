#include "coverage/imu_sample_buffer.h"

#include <algorithm>
#include <cstdint>

namespace causal_slam::coverage {

ImuSampleBuffer::ImuSampleBuffer(std::int64_t retention_ns)
    : retention_ns_(std::max<std::int64_t>(retention_ns, 0)) {}

void ImuSampleBuffer::Add(ImuSample sample) {
  if (!samples_.empty() && sample.stamp_ns < samples_.back().stamp_ns) {
    ++dropped_out_of_order_count_;
    return;
  }

  samples_.push_back(sample);
  PruneOlderThan(sample.stamp_ns - retention_ns_);
}

void ImuSampleBuffer::PruneOlderThan(std::int64_t cutoff_ns) {
  while (!samples_.empty() && samples_.front().stamp_ns < cutoff_ns) {
    samples_.pop_front();
  }
}

const std::deque<ImuSample>& ImuSampleBuffer::Samples() const {
  return samples_;
}

std::size_t ImuSampleBuffer::Size() const {
  return samples_.size();
}

std::uint64_t ImuSampleBuffer::DroppedOutOfOrderCount() const {
  return dropped_out_of_order_count_;
}

}  // namespace causal_slam::coverage