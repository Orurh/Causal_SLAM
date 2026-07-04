#include "domain/diagnostics/temporal_fault_reason_formatter.h"

#include <gtest/gtest.h>

namespace causal_slam::diagnostics {
namespace {

TemporalDiagnosticIssue MakeIssue(TemporalDiagnosticSeverity severity, TemporalFaultReason reason, const std::string& title) {
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

TEST(TemporalFaultReasonFormatterTest, FaultReasonStringsAreStable) {
  EXPECT_STREQ(ToString(TemporalFaultReason::kNoLidarScanReceivedYet), "no_lidar_scan_received_yet");
  EXPECT_STREQ(ToString(TemporalFaultReason::kNoImuSampleReceivedYet), "no_imu_sample_received_yet");
  EXPECT_STREQ(ToString(TemporalFaultReason::kLidarStreamStale), "lidar_stream_stale");
  EXPECT_STREQ(ToString(TemporalFaultReason::kImuStreamStale), "imu_stream_stale");
}

TEST(TemporalFaultReasonFormatterTest, StaleStreamReasonsAreJoined) {
  const std::vector<TemporalDiagnosticIssue> issues{
      MakeIssue(TemporalDiagnosticSeverity::kDegraded, TemporalFaultReason::kLidarStreamStale, "lidar stale"),
      MakeIssue(TemporalDiagnosticSeverity::kDegraded, TemporalFaultReason::kImuStreamStale, "imu stale"),
  };

  EXPECT_EQ(JoinFaultReasons(issues), "lidar_stream_stale,imu_stream_stale");
}

TEST(TemporalFaultReasonFormatterTest, SingleIssueProducesReason) {
  const std::vector<TemporalDiagnosticIssue> issues{
      MakeIssue(TemporalDiagnosticSeverity::kDegraded, TemporalFaultReason::kImuWindowIncomplete, "test"),
  };

  EXPECT_EQ(JoinFaultReasons(issues), "imu_window_incomplete");
}

TEST(TemporalFaultReasonFormatterTest, MultipleIssuesAreCommaSeparated) {
  const std::vector<TemporalDiagnosticIssue> issues{
      MakeIssue(TemporalDiagnosticSeverity::kDegraded, TemporalFaultReason::kStreamTimingUnstable, "stream"),
      MakeIssue(TemporalDiagnosticSeverity::kDegraded, TemporalFaultReason::kImuWindowIncomplete, "coverage"),
  };

  EXPECT_EQ(JoinFaultReasons(issues), "stream_timing_unstable,imu_window_incomplete");
}

}  // namespace
}  // namespace causal_slam::diagnostics
