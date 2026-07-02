#include "domain/policy/map_update_decision.h"

#include <gtest/gtest.h>

namespace causal_slam::policy {
namespace {

namespace telemetry = causal_slam::telemetry;

TEST(MapUpdateDecisionTest, AllowsOkAndWarning) {
  const auto ok = DecideMapUpdate(telemetry::TemporalHealthStatus::kOk);
  EXPECT_TRUE(ok.map_update_allowed);
  EXPECT_EQ(ok.reason, MapUpdateDecisionReason::kTemporalHealthOk);

  const auto warning = DecideMapUpdate(telemetry::TemporalHealthStatus::kWarning);
  EXPECT_TRUE(warning.map_update_allowed);
  EXPECT_EQ(warning.reason, MapUpdateDecisionReason::kTemporalHealthWarning);
}

TEST(MapUpdateDecisionTest, RejectsDegradedAndInvalid) {
  const auto degraded = DecideMapUpdate(telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_FALSE(degraded.map_update_allowed);
  EXPECT_EQ(degraded.reason, MapUpdateDecisionReason::kTemporalHealthDegraded);

  const auto invalid = DecideMapUpdate(telemetry::TemporalHealthStatus::kInvalid);
  EXPECT_FALSE(invalid.map_update_allowed);
  EXPECT_EQ(invalid.reason, MapUpdateDecisionReason::kTemporalHealthInvalid);
}

TEST(MapUpdateDecisionTest, ReasonStringsAreStable) {
  EXPECT_STREQ(ToString(MapUpdateDecisionReason::kTemporalHealthOk), "temporal_health_ok");
  EXPECT_STREQ(ToString(MapUpdateDecisionReason::kTemporalHealthWarning), "temporal_health_warning");
  EXPECT_STREQ(ToString(MapUpdateDecisionReason::kTemporalHealthDegraded), "temporal_health_degraded");
  EXPECT_STREQ(ToString(MapUpdateDecisionReason::kTemporalHealthInvalid), "temporal_health_invalid");
}

}  // namespace
}  // namespace causal_slam::policy
