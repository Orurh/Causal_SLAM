#include "domain/telemetry/stream_staleness_analyzer.h"

#include <optional>

#include <gtest/gtest.h>

namespace causal_slam::telemetry {
namespace {

StreamStalenessConfig MakeConfig(bool enabled) {
  StreamStalenessConfig config;
  config.enabled = enabled;
  config.max_staleness_ns = 500'000'000LL;
  return config;
}


TEST(StreamStalenessAnalyzerTest, MissingStreamIsReportedAsMissing) {
  StreamStalenessAnalyzer analyzer(
      TemporalStreamId::kLidar,
      MakeConfig(true));

  const auto summary = analyzer.Analyze(std::nullopt, 1'000'000'000LL);

  EXPECT_EQ(summary.stream_id, TemporalStreamId::kLidar);
  EXPECT_TRUE(summary.IsMissing());
  EXPECT_FALSE(summary.IsFresh());
  EXPECT_FALSE(summary.IsStale());
}

TEST(StreamStalenessAnalyzerTest, FreshStreamIsReportedAsFresh) {
  StreamStalenessAnalyzer analyzer(
      TemporalStreamId::kImu,
      MakeConfig(true));

  const auto summary = analyzer.Analyze(1'000'000'000LL, 1'200'000'000LL);

  EXPECT_EQ(summary.stream_id, TemporalStreamId::kImu);
  EXPECT_TRUE(summary.IsFresh());
  EXPECT_EQ(summary.age_ns, 200'000'000LL);
}

TEST(StreamStalenessAnalyzerTest, StaleStreamIsReportedAsStale) {
  StreamStalenessAnalyzer analyzer(
      TemporalStreamId::kLidar,
      MakeConfig(true));

  const auto summary = analyzer.Analyze(1'000'000'000LL, 1'700'000'000LL);

  EXPECT_TRUE(summary.IsStale());
  EXPECT_EQ(summary.age_ns, 700'000'000LL);
}

TEST(StreamStalenessAnalyzerTest, DisabledAnalyzerTreatsStreamAsFresh) {
  StreamStalenessAnalyzer analyzer(
      TemporalStreamId::kLidar,
      MakeConfig(false));

  const auto summary = analyzer.Analyze(std::nullopt, 1'000'000'000LL);

  EXPECT_TRUE(summary.IsFresh());
  EXPECT_FALSE(summary.IsStale());
  EXPECT_FALSE(summary.IsMissing());
}

TEST(StreamStalenessAnalyzerTest, NegativeAgeIsClampedToZero) {
  StreamStalenessAnalyzer analyzer(
      TemporalStreamId::kLidar,
      MakeConfig(true));

  const auto summary = analyzer.Analyze(2'000'000'000LL, 1'000'000'000LL);

  EXPECT_TRUE(summary.IsFresh());
  EXPECT_EQ(summary.age_ns, 0);
}

}  // namespace
}  // namespace causal_slam::telemetry