#include "presentation/render/console_temporal_summary_renderer.h"

#include <string>

#include <gtest/gtest.h>

#include "policy/map_update_decision.h"

namespace causal_slam::render {
namespace {

namespace diagnostics = causal_slam::diagnostics;
namespace policy = causal_slam::policy;
namespace telemetry = causal_slam::telemetry;
namespace transform = causal_slam::transform;

diagnostics::TemporalDiagnosticSnapshot MakeSnapshot() {
  diagnostics::TemporalDiagnosticSnapshot snapshot;
  snapshot.overall_status = telemetry::TemporalHealthStatus::kOk;
  snapshot.map_update_decision =
      policy::DecideMapUpdate(telemetry::TemporalHealthStatus::kOk);
  return snapshot;
}

TEST(ConsoleTemporalSummaryRendererTest, RendersNoTfChecksWhenEmpty) {
  const ConsoleTemporalSummaryRenderer renderer;

  const auto text = renderer.Render(MakeSnapshot());

  EXPECT_NE(text.find("TF checks:"), std::string::npos);
  EXPECT_NE(text.find("  none"), std::string::npos);
}

TEST(ConsoleTemporalSummaryRendererTest, RendersTransformAgeSummary) {
  auto snapshot = MakeSnapshot();

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

  const ConsoleTemporalSummaryRenderer renderer;
  const auto text = renderer.Render(snapshot);

  EXPECT_NE(text.find("TF checks:"), std::string::npos);
  EXPECT_NE(text.find("odom <- lidar: INVALID"), std::string::npos);
  EXPECT_NE(text.find("status=tf_lookup_failed"), std::string::npos);
  EXPECT_NE(text.find("receive_delay_ms=12.5"), std::string::npos);
  EXPECT_NE(text.find("detail=frame does not exist"), std::string::npos);
}

}  // namespace
}  // namespace causal_slam::render
