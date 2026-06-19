#include "telemetry/stream_timing_tracker.h"

#include <cstdint>

#include <gtest/gtest.h>

namespace causal_slam::telemetry {
namespace {

constexpr std::int64_t kDelayNs = 300'000;  // 0.3 ms

std::int64_t Ms(const std::int64_t milliseconds) {
  return milliseconds * 1'000'000;
}

TimingSample SampleAtMs(const std::int64_t stamp_ms) {
  const std::int64_t stamp_ns = Ms(stamp_ms);
  return TimingSample{
      .header_stamp_ns = stamp_ns,
      .receive_time_ns = stamp_ns + kDelayNs,
  };
}

TEST(StreamTimingTrackerTest, RegularHundredHzWindowIsOk) {
  StreamTimingTracker tracker;

  for (int i = 0; i < 200; ++i) {
    tracker.Observe(SampleAtMs(i * 10));
  }

  const TimingSummary summary = tracker.ConsumeWindowSummary();

  EXPECT_EQ(summary.total_count, 200u);
  EXPECT_EQ(summary.window_count, 200u);

  EXPECT_NEAR(summary.last_delay_ms, 0.3, 1e-9);
  EXPECT_NEAR(summary.window_average_delay_ms, 0.3, 1e-9);
  EXPECT_NEAR(summary.window_max_delay_ms, 0.3, 1e-9);

  EXPECT_NEAR(summary.last_period_ms, 10.0, 1e-9);
  EXPECT_NEAR(summary.window_max_jitter_ms, 0.0, 1e-9);

  EXPECT_EQ(summary.total_gap_count, 0u);
  EXPECT_EQ(summary.window_gap_count, 0u);
  EXPECT_EQ(summary.total_reordered_count, 0u);
  EXPECT_EQ(summary.window_reordered_count, 0u);

  EXPECT_EQ(summary.health, TimingHealth::kOk);
  EXPECT_EQ(summary.reason, "ok");
}

TEST(StreamTimingTrackerTest, GapDegradesOnlyCurrentWindowThenRecovers) {
  StreamTimingTracker tracker;

  tracker.Observe(SampleAtMs(0));
  tracker.Observe(SampleAtMs(10));
  tracker.Observe(SampleAtMs(20));

  const TimingSummary first_window = tracker.ConsumeWindowSummary();
  EXPECT_EQ(first_window.health, TimingHealth::kOk);
  EXPECT_EQ(first_window.total_gap_count, 0u);
  EXPECT_EQ(first_window.window_gap_count, 0u);

  tracker.Observe(SampleAtMs(30));
  tracker.Observe(SampleAtMs(1700));

  const TimingSummary gap_window = tracker.ConsumeWindowSummary();
  EXPECT_EQ(gap_window.health, TimingHealth::kDegraded);
  EXPECT_EQ(gap_window.reason, "stream_gap");
  EXPECT_EQ(gap_window.total_gap_count, 1u);
  EXPECT_EQ(gap_window.window_gap_count, 1u);
  EXPECT_NEAR(gap_window.max_gap_ms, 1670.0, 1e-9);

  tracker.Observe(SampleAtMs(1710));
  tracker.Observe(SampleAtMs(1720));
  tracker.Observe(SampleAtMs(1730));

  const TimingSummary recovered_window = tracker.ConsumeWindowSummary();
  EXPECT_EQ(recovered_window.health, TimingHealth::kOk);
  EXPECT_EQ(recovered_window.reason, "ok");
  EXPECT_EQ(recovered_window.total_gap_count, 1u);
  EXPECT_EQ(recovered_window.window_gap_count, 0u);
}

TEST(StreamTimingTrackerTest, ReorderedTimestampDegradesWindow) {
  StreamTimingTracker tracker;

  tracker.Observe(SampleAtMs(20));
  tracker.Observe(SampleAtMs(10));

  const TimingSummary summary = tracker.ConsumeWindowSummary();

  EXPECT_EQ(summary.total_reordered_count, 1u);
  EXPECT_EQ(summary.window_reordered_count, 1u);
  EXPECT_EQ(summary.health, TimingHealth::kDegraded);
  EXPECT_EQ(summary.reason, "message_reordering_detected");
}

TEST(StreamTimingTrackerTest, SuspiciousJitterProducesWarning) {
  StreamTimingTracker tracker;

  tracker.Observe(SampleAtMs(0));
  tracker.Observe(SampleAtMs(10));
  tracker.Observe(SampleAtMs(25));

  const TimingSummary summary = tracker.ConsumeWindowSummary();

  EXPECT_NEAR(summary.window_max_jitter_ms, 5.0, 1e-9);
  EXPECT_EQ(summary.health, TimingHealth::kWarning);
  EXPECT_EQ(summary.reason, "jitter_suspicious");
}

TEST(StreamTimingTrackerTest, HighJitterDegradesWindow) {
  StreamTimingTracker tracker;

  tracker.Observe(SampleAtMs(0));
  tracker.Observe(SampleAtMs(10));
  tracker.Observe(SampleAtMs(35));

  const TimingSummary summary = tracker.ConsumeWindowSummary();

  EXPECT_NEAR(summary.window_max_jitter_ms, 15.0, 1e-9);
  EXPECT_EQ(summary.health, TimingHealth::kDegraded);
  EXPECT_EQ(summary.reason, "jitter_high");
}

TEST(StreamTimingTrackerTest, CustomGapThresholdIsUsed) {
  StreamTimingTracker tracker;
  tracker.SetGapThresholdMs(30.0);

  tracker.Observe(SampleAtMs(0));
  tracker.Observe(SampleAtMs(10));
  tracker.Observe(SampleAtMs(50));

  const TimingSummary summary = tracker.ConsumeWindowSummary();

  EXPECT_EQ(summary.total_gap_count, 1u);
  EXPECT_EQ(summary.window_gap_count, 1u);
  EXPECT_NEAR(summary.max_gap_ms, 40.0, 1e-9);
  EXPECT_EQ(summary.health, TimingHealth::kDegraded);
  EXPECT_EQ(summary.reason, "stream_gap");
}

}  // namespace
}  // namespace causal_slam::telemetry