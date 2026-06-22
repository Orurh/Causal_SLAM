#include "diagnostics/temporal_fault_reason_formatter.h"

#include <gtest/gtest.h>

namespace causal_slam::diagnostics {
namespace {

TEST(TemporalFaultReasonFormatterTest, EmptyIssuesProducesNone) {
  EXPECT_EQ(JoinFaultReasons({}), "none");
}

TEST(TemporalFaultReasonFormatterTest, SingleIssueProducesReason) {
  const std::vector<TemporalDiagnosticIssue> issues{
      TemporalDiagnosticIssue{
          .severity = TemporalDiagnosticSeverity::kDegraded,
          .reason = TemporalFaultReason::kImuWindowIncomplete,
          .title = "test",
      },
  };

  EXPECT_EQ(JoinFaultReasons(issues), "imu_window_incomplete");
}

TEST(TemporalFaultReasonFormatterTest, MultipleIssuesAreCommaSeparated) {
  const std::vector<TemporalDiagnosticIssue> issues{
      TemporalDiagnosticIssue{
          .severity = TemporalDiagnosticSeverity::kDegraded,
          .reason = TemporalFaultReason::kStreamTimingUnstable,
          .title = "stream",
      },
      TemporalDiagnosticIssue{
          .severity = TemporalDiagnosticSeverity::kDegraded,
          .reason = TemporalFaultReason::kImuWindowIncomplete,
          .title = "coverage",
      },
  };

  EXPECT_EQ(JoinFaultReasons(issues),
            "stream_timing_unstable,imu_window_incomplete");
}

}  // namespace
}  // namespace causal_slam::diagnostics
