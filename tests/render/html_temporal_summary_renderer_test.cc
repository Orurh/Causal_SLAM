#include "presentation/render/html_temporal_summary_renderer.h"

#include <string>

#include <gtest/gtest.h>

#include "domain/policy/map_update_decision.h"

namespace causal_slam::render {
namespace {

namespace diagnostics = causal_slam::diagnostics;
namespace policy = causal_slam::policy;
namespace telemetry = causal_slam::telemetry;
namespace transform = causal_slam::transform;

diagnostics::TemporalDiagnosticSnapshot MakeSnapshot() {
  diagnostics::TemporalDiagnosticSnapshot snapshot;
  snapshot.overall_status = telemetry::TemporalHealthStatus::kOk;
  return snapshot;
}

causal_slam::policy::MapUpdateDecision MakeDecision() {
  return causal_slam::policy::DecideMapUpdate(causal_slam::telemetry::TemporalHealthStatus::kOk);
}

TEST(HtmlTemporalSummaryRendererTest, RendersStandaloneHtmlPage) {
  const HtmlTemporalSummaryRenderer renderer;

  const auto html = renderer.RenderPage(MakeSnapshot(), MakeDecision(), causal_slam::statistics::TemporalStatisticsSnapshot{});

  EXPECT_NE(html.find("<!doctype html>"), std::string::npos);
  EXPECT_NE(html.find("<title>Causal-SLAM Temporal Report</title>"), std::string::npos);
  EXPECT_NE(html.find("Temporal Health"), std::string::npos);
  EXPECT_NE(html.find("Map update"), std::string::npos);
  EXPECT_NE(html.find("Causal-SLAM Temporal Report"), std::string::npos);
  EXPECT_NE(html.find("<div class=\"grid\">"), std::string::npos);
  EXPECT_EQ(html.find("<h1>Temporal Statistics</h1>"), std::string::npos);
}

TEST(HtmlTemporalSummaryRendererTest, EscapesUnsafeText) {
  auto snapshot = MakeSnapshot();

  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kInvalid;
  summary.status = transform::TransformLookupStatus::kLookupFailed;
  summary.target_frame = "odom<script>";
  summary.source_frame = "lidar";
  summary.reason = "tf_lookup_failed";
  summary.adapter_detail = "bad <frame> & \"quoted\"";

  snapshot.observation.transform_ages.push_back(summary);

  const HtmlTemporalSummaryRenderer renderer;
  const auto html = renderer.RenderDiagnostics(snapshot, MakeDecision());

  EXPECT_EQ(html.find("odom<script>"), std::string::npos);
  EXPECT_NE(html.find("odom&lt;script&gt;"), std::string::npos);
  EXPECT_NE(html.find("bad &lt;frame&gt; &amp; &quot;quoted&quot;"), std::string::npos);
}

TEST(HtmlTemporalSummaryRendererTest, RendersTfChecksFromReportDocument) {
  auto snapshot = MakeSnapshot();

  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kOk;
  summary.status = transform::TransformLookupStatus::kOk;
  summary.target_frame = "odom";
  summary.source_frame = "lidar";
  summary.transform_age_ms = 0.0;
  summary.receive_delay_ms = 3.5;

  snapshot.observation.transform_ages.push_back(summary);

  const HtmlTemporalSummaryRenderer renderer;
  const auto html = renderer.RenderDiagnostics(snapshot, MakeDecision());

  EXPECT_NE(html.find("TF checks"), std::string::npos);
  EXPECT_NE(html.find("odom &lt;- lidar"), std::string::npos);
  EXPECT_NE(html.find("status-ok"), std::string::npos);
}

}  // namespace
}  // namespace causal_slam::render
