#include "coverage/imu_sample_buffer.h"

#include <cstdint>

#include <gtest/gtest.h>

namespace causal_slam::coverage {
namespace {

std::int64_t Ms(const std::int64_t milliseconds) {
  return milliseconds * 1'000'000;
}

ImuSample ImuAtMs(const std::int64_t timestamp_ms) {
  return ImuSample{
      .stamp_ns = Ms(timestamp_ms),
  };
}

TEST(ImuSampleBufferTest, KeepsSamplesWithinRetentionWindow) {
  ImuSampleBuffer buffer{Ms(100)};

  buffer.Add(ImuAtMs(1000));
  buffer.Add(ImuAtMs(1040));
  buffer.Add(ImuAtMs(1080));
  buffer.Add(ImuAtMs(1120));

  EXPECT_EQ(buffer.Size(), 3u);
  ASSERT_EQ(buffer.Samples().size(), 3u);

  EXPECT_EQ(buffer.Samples()[0].stamp_ns, Ms(1040));
  EXPECT_EQ(buffer.Samples()[1].stamp_ns, Ms(1080));
  EXPECT_EQ(buffer.Samples()[2].stamp_ns, Ms(1120));
}

TEST(ImuSampleBufferTest, ManualPruneRemovesOldSamples) {
  ImuSampleBuffer buffer{Ms(1000)};

  buffer.Add(ImuAtMs(1000));
  buffer.Add(ImuAtMs(1100));
  buffer.Add(ImuAtMs(1200));

  buffer.PruneOlderThan(Ms(1100));

  EXPECT_EQ(buffer.Size(), 2u);
  ASSERT_EQ(buffer.Samples().size(), 2u);

  EXPECT_EQ(buffer.Samples()[0].stamp_ns, Ms(1100));
  EXPECT_EQ(buffer.Samples()[1].stamp_ns, Ms(1200));
}

TEST(ImuSampleBufferTest, DropsOutOfOrderSamples) {
  ImuSampleBuffer buffer{Ms(1000)};

  buffer.Add(ImuAtMs(1000));
  buffer.Add(ImuAtMs(1020));
  buffer.Add(ImuAtMs(1010));
  buffer.Add(ImuAtMs(1040));

  EXPECT_EQ(buffer.Size(), 3u);
  EXPECT_EQ(buffer.DroppedOutOfOrderCount(), 1u);

  ASSERT_EQ(buffer.Samples().size(), 3u);
  EXPECT_EQ(buffer.Samples()[0].stamp_ns, Ms(1000));
  EXPECT_EQ(buffer.Samples()[1].stamp_ns, Ms(1020));
  EXPECT_EQ(buffer.Samples()[2].stamp_ns, Ms(1040));
}

}  // namespace
}  // namespace causal_slam::coverage
