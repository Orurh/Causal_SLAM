#include "domain/diagnostics/temporal_fault_reason_formatter.h"

#include <gtest/gtest.h>

namespace causal_slam::diagnostics {
namespace {

TemporalDiagnosticIssue MakeIssue(
    TemporalDiagnosticSeverity severity,
    TemporalFaultReason reason,
    const std::string& title) {
  TemporalDiagnosticIssue issue;
  issue.severity = severity;
  issue.reason = reason;
  issue.title = title;
  issue.explanation = "test explanation";
  issue.evidence = "test evidence";
  issue.suggested_action = "test action";
  return issue;
}

TEST(TemporalFaultReasonFormatterTest, EmptyIssuesProducesNone) {
  EXPECT_EQ(JoinFaultReasons({}), "none");
}

TEST(TemporalFaultReasonFormatterTest, SingleIssueProducesReason) {
  const std::vector<TemporalDiagnosticIssue> issues{
      MakeIssue(TemporalDiagnosticSeverity::kDegraded,
                TemporalFaultReason::kImuWindowIncomplete,
                "test"),
  };

  EXPECT_EQ(JoinFaultReasons(issues), "imu_window_incomplete");
}

TEST(TemporalFaultReasonFormatterTest, MultipleIssuesAreCommaSeparated) {
  const std::vector<TemporalDiagnosticIssue> issues{
      MakeIssue(TemporalDiagnosticSeverity::kDegraded,
                TemporalFaultReason::kStreamTimingUnstable,
                "stream"),
      MakeIssue(TemporalDiagnosticSeverity::kDegraded,
                TemporalFaultReason::kImuWindowIncomplete,
                "coverage"),
  };

  EXPECT_EQ(JoinFaultReasons(issues),
            "stream_timing_unstable,imu_window_incomplete");
}

}  // namespace
}  // namespace causal_slam::diagnostics
