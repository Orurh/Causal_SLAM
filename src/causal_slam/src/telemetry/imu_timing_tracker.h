#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace causal_slam::telemetry {

struct ImuTimingSample {
  std::int64_t header_stamp_ns{0};
  std::int64_t receive_time_ns{0};
};

enum class TimingHealth {
  kOk,
  kWarning,
  kDegraded,
};

[[nodiscard]] const char* ToString(TimingHealth health);

struct ImuTimingSummary {
  std::uint64_t total_count{0};
  std::uint64_t window_count{0};

  double last_delay_ms{0.0};
  double window_average_delay_ms{0.0};
  double window_max_delay_ms{0.0};

  double last_period_ms{0.0};
  double last_jitter_ms{0.0};
  double window_max_jitter_ms{0.0};

  std::uint64_t total_reordered_count{0};
  std::uint64_t window_reordered_count{0};

  std::uint64_t total_gap_count{0};
  std::uint64_t window_gap_count{0};
  double max_gap_ms{0.0};

  TimingHealth health{TimingHealth::kOk};
  std::string reason{"ok"};
};

class ImuTimingTracker final {
 public:
  void Observe(const ImuTimingSample& sample);
  [[nodiscard]] ImuTimingSummary LifetimeSummary() const;
  [[nodiscard]] ImuTimingSummary ConsumeWindowSummary();
 
 private:
  void ResetWindow();

  std::uint64_t count_{0};

  double last_delay_ms_{0.0};
  double delay_sum_ms_{0.0};
  double max_delay_ms_{0.0};

  std::optional<std::int64_t> previous_stamp_ns_;
  std::optional<std::int64_t> previous_period_ns_;

  double last_period_ms_{0.0};
  double last_jitter_ms_{0.0};
  double max_jitter_ms_{0.0};

  std::uint64_t reordered_count_{0};

  std::uint64_t gap_count_{0};
  double max_gap_ms_{0.0};

  std::uint64_t window_count_{0};
  double window_delay_sum_ms_{0.0};
  double window_max_delay_ms_{0.0};
  double window_max_jitter_ms_{0.0};
  std::uint64_t window_reordered_count_{0};
  std::uint64_t window_gap_count_{0};
};

}  // namespace causal_slam::telemetry