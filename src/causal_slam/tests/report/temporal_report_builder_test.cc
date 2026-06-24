#include "presentation/report/temporal_report_builder.h"

#include <algorithm>
#include <string>

#include <gtest/gtest.h>

#include "policy/map_update_decision.h"

namespace causal_slam::report {
namespace {

namespace diagnostics = causal_slam::diagnostics;
namespace policy = causal_slam::policy;
namespace telemetry = causal_slam::telemetry;
namespace transform = causal_slam::transform;

const ReportSection* FindSection(
    const ReportDocument& document,
    const std::string& id) {
  const auto it = std::find_if(
      document.sections.begin(),
      document.sections.end(),
      [&](const ReportSection& section) { return section.id == id; });

  if (it == document.sections.end()) {
    return nullptr;
  }

  return &*it;
}

diagnostics::TemporalDiagnosticSnapshot MakeDiagnosticSnapshot() {
  diagnostics::TemporalDiagnosticSnapshot snapshot;
  snapshot.overall_status = telemetry::TemporalHealthStatus::kOk;
  snapshot.map_update_decision =
      policy::DecideMapUpdate(telemetry::TemporalHealthStatus::kOk);
  return snapshot;
}

TEST(TemporalReportBuilderTest, BuildsTransformChecksSection) {
  auto snapshot = MakeDiagnosticSnapshot();

  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kInvalid;
  summary.status = transform::TransformLookupStatus::kLookupFailed;
  summary.target_frame = "odom";
  summary.source_frame = "lidar";
  summary.transform_age_ms = 0.0;
  summary.receive_delay_ms = 12.5;
  summary.reason = "tf_lookup_failed";
  summary.adapter_detail = "frame does not exist";

  snapshot.observation.transform_ages.push_back(summary);

  const TemporalReportBuilder builder;
  const auto document = builder.BuildDiagnosticsReport(snapshot);

  const auto* section = FindSection(document, "tf_checks");
  ASSERT_NE(section, nullptr);

  ASSERT_EQ(section->rows.size(), 1U);
  EXPECT_EQ(section->rows[0].label, "odom <- lidar");
  EXPECT_EQ(section->rows[0].status, "INVALID");
  EXPECT_EQ(section->rows[0].reason, "tf_lookup_failed");
  EXPECT_EQ(section->rows[0].detail, "frame does not exist");
}

TEST(TemporalReportBuilderTest, BuildsStatisticsWindowsAsSections) {
  const TemporalReportBuilder builder;
  const auto document =
      builder.BuildStatisticsReport(causal_slam::statistics::TemporalStatisticsSnapshot{});

  EXPECT_EQ(document.title, "Temporal Statistics (historical windows)");
  EXPECT_NE(FindSection(document, "last_10s"), nullptr);
  EXPECT_NE(FindSection(document, "last_60s"), nullptr);
  EXPECT_NE(FindSection(document, "session"), nullptr);
}

}  // namespace
}  // namespace causal_slam::report
