#include "imu_timing_tracker.h"

#include <algorithm>
#include <cstdint>

namespace causal_slam::telemetry {

namespace {
constexpr std::int64_t kLargeImuGapNs = 100'000'000;  // 100 ms

double NanosecondsToMilliseconds(const std::int64_t nanoseconds) {
  return static_cast<double>(nanoseconds) / 1'000'000.0;
}

std::int64_t AbsNanoseconds(const std::int64_t value) {
  return value < 0 ? -value : value;
}

ImuTimingSummary EvaluateHealth(ImuTimingSummary summary) {
  if (summary.window_reordered_count > 0) {
    summary.health = TimingHealth::kDegraded;
    summary.reason = "imu_message_reordering_detected";
    return summary;
  }

  if (summary.window_gap_count > 0) {
    summary.health = TimingHealth::kDegraded;
    summary.reason = "imu_stream_gap";
    return summary;
  }

  if (summary.window_max_jitter_ms > 10.0) {
    summary.health = TimingHealth::kDegraded;
    summary.reason = "imu_jitter_high";
    return summary;
  }

  if (summary.window_max_jitter_ms > 2.0) {
    summary.health = TimingHealth::kWarning;
    summary.reason = "imu_jitter_suspicious";
    return summary;
  }

  summary.health = TimingHealth::kOk;
  summary.reason = "ok";
  return summary;
}

}  // namespace

const char* ToString(const TimingHealth health) {
  switch (health) {
    case TimingHealth::kOk:
      return "OK";
    case TimingHealth::kWarning:
      return "WARNING";
    case TimingHealth::kDegraded:
      return "DEGRADED";
  }

  return "UNKNOWN";
}

void ImuTimingTracker::Observe(const ImuTimingSample& sample) {
  const std::int64_t delay_ns = sample.receive_time_ns - sample.header_stamp_ns;

  last_delay_ms_ = NanosecondsToMilliseconds(delay_ns);
  delay_sum_ms_ += last_delay_ms_;
  max_delay_ms_ = std::max(max_delay_ms_, last_delay_ms_);
  window_delay_sum_ms_ += last_delay_ms_;
  window_max_delay_ms_ = std::max(window_max_delay_ms_, last_delay_ms_);

  if (previous_stamp_ns_.has_value()) {
    const std::int64_t period_ns = sample.header_stamp_ns - *previous_stamp_ns_;

    if (period_ns < 0) {
      ++reordered_count_;
      ++window_reordered_count_;
    } else if (period_ns > kLargeImuGapNs) {
      ++gap_count_;
      ++window_gap_count_;

      const double gap_ms = NanosecondsToMilliseconds(period_ns);
      max_gap_ms_ = std::max(max_gap_ms_, gap_ms);

      last_period_ms_ = gap_ms;
      previous_period_ns_.reset();
    } else {
      last_period_ms_ = NanosecondsToMilliseconds(period_ns);

      if (previous_period_ns_.has_value()) {
        const std::int64_t jitter_ns = AbsNanoseconds(period_ns - *previous_period_ns_);
        last_jitter_ms_ = NanosecondsToMilliseconds(jitter_ns);
        max_jitter_ms_ = std::max(max_jitter_ms_, last_jitter_ms_);
        window_max_jitter_ms_ = std::max(window_max_jitter_ms_, last_jitter_ms_);
      }

      previous_period_ns_ = period_ns;
    }
  }

  previous_stamp_ns_ = sample.header_stamp_ns;
  ++count_;
  ++window_count_;
}

ImuTimingSummary ImuTimingTracker::LifetimeSummary() const {
  const double average_delay_ms = count_ > 0 ? delay_sum_ms_ / static_cast<double>(count_) : 0.0;

  auto summary = ImuTimingSummary{
      .total_count = count_,
      .window_count = count_,
      .last_delay_ms = last_delay_ms_,
      .window_average_delay_ms = average_delay_ms,
      .window_max_delay_ms = max_delay_ms_,
      .last_period_ms = last_period_ms_,
      .last_jitter_ms = last_jitter_ms_,
      .window_max_jitter_ms = max_jitter_ms_,
      .total_reordered_count = reordered_count_,
      .window_reordered_count = reordered_count_,
      .total_gap_count = gap_count_,
      .window_gap_count = gap_count_,
      .max_gap_ms = max_gap_ms_,
  };
  return EvaluateHealth(std::move(summary));
}

ImuTimingSummary ImuTimingTracker::ConsumeWindowSummary() {
  const double window_average_delay_ms = window_count_ > 0 ? window_delay_sum_ms_ / static_cast<double>(window_count_) : 0.0;

  auto summary = ImuTimingSummary{
      .total_count = count_,
      .window_count = window_count_,
      .last_delay_ms = last_delay_ms_,
      .window_average_delay_ms = window_average_delay_ms,
      .window_max_delay_ms = window_max_delay_ms_,
      .last_period_ms = last_period_ms_,
      .last_jitter_ms = last_jitter_ms_,
      .window_max_jitter_ms = window_max_jitter_ms_,
      .total_reordered_count = reordered_count_,
      .window_reordered_count = window_reordered_count_,
      .total_gap_count = gap_count_,
      .window_gap_count = window_gap_count_,
      .max_gap_ms = max_gap_ms_,
  };

  summary = EvaluateHealth(std::move(summary));
  ResetWindow();
  return summary;
}

void ImuTimingTracker::ResetWindow() {
  window_count_ = 0;
  window_delay_sum_ms_ = 0.0;
  window_max_delay_ms_ = 0.0;
  window_max_jitter_ms_ = 0.0;
  window_reordered_count_ = 0;
  window_gap_count_ = 0;
}

}  // namespace causal_slam::telemetry