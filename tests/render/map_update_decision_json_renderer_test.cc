#include "presentation/render/map_update_decision_json_renderer.h"

#include <gtest/gtest.h>

#include "domain/diagnostics/temporal_diagnostics.h"
#include "domain/policy/map_update_decision.h"

namespace causal_slam::render {
namespace {

TEST(MapUpdateDecisionJsonRendererTest, RendersScanScopedDecision) {
  causal_slam::diagnostics::TemporalDiagnosticSnapshot snapshot;
  snapshot.overall_status = causal_slam::telemetry::TemporalHealthStatus::kDegraded;

  causal_slam::policy::MapUpdateDecision map_update_decision;
  map_update_decision.map_update_allowed = false;
  map_update_decision.reason =
      causal_slam::policy::MapUpdateDecisionReason::
          kTemporalHealthDegraded;

  snapshot.observation.has_lidar_scan = true;
  snapshot.observation.latest_lidar_header_stamp_ns = 123456789;
  snapshot.observation.latest_lidar_frame_id = "lidar";
  causal_slam::diagnostics::TemporalDiagnosticIssue issue;
  issue.severity =
      causal_slam::diagnostics::TemporalDiagnosticSeverity::kDegraded;
  issue.reason =
      causal_slam::diagnostics::TemporalFaultReason::
          kImuWindowIncomplete;
  issue.title = "bad imu";
  issue.explanation = "x";
  issue.evidence = "missing_prefix_ms=12";
  issue.suggested_action = "fix timing";
  snapshot.issues.push_back(issue);

  const auto json = RenderMapUpdateDecisionJson(snapshot, map_update_decision);

  EXPECT_NE(json.find("\"has_lidar_scan\":true"), std::string::npos);
  EXPECT_NE(json.find("\"scan_stamp_ns\":123456789"), std::string::npos);
  EXPECT_NE(json.find("\"frame_id\":\"lidar\""), std::string::npos);
  EXPECT_NE(json.find("\"allowed\":false"), std::string::npos);
  EXPECT_NE(json.find("\"health\":\"DEGRADED\""), std::string::npos);
  EXPECT_NE(json.find("\"reason\":\"temporal_health_degraded\""), std::string::npos);
  EXPECT_NE(json.find("\"fault_reasons\":[\"imu_window_incomplete\"]"), std::string::npos);
  EXPECT_NE(json.find("missing_prefix_ms=12"), std::string::npos);
}

TEST(MapUpdateDecisionJsonRendererTest, RendersStreamTimingFaultReason) {
  causal_slam::diagnostics::TemporalDiagnosticSnapshot snapshot;
  snapshot.overall_status = causal_slam::telemetry::TemporalHealthStatus::kDegraded;
  snapshot.observation.has_lidar_scan = true;
  snapshot.observation.latest_lidar_header_stamp_ns = 987654321;
  snapshot.observation.latest_lidar_frame_id = "os1_lidar";

  causal_slam::diagnostics::TemporalDiagnosticIssue issue;
  issue.severity =
      causal_slam::diagnostics::TemporalDiagnosticSeverity::kDegraded;
  issue.reason =
      causal_slam::diagnostics::TemporalFaultReason::
          kLidarStreamTimingJitterHigh;
  issue.title = "LiDAR stream timing is not stable";
  issue.explanation =
      "The LiDAR message stream has high timing jitter.";
  issue.evidence =
      "stream=lidar, reason=jitter_high, "
      "window_max_jitter_ms=12.5";
  issue.suggested_action =
      "Check sensor driver timing and timestamp source.";
  snapshot.issues.push_back(issue);

  causal_slam::policy::MapUpdateDecision map_update_decision;
  map_update_decision.map_update_allowed = false;
  map_update_decision.reason =
      causal_slam::policy::MapUpdateDecisionReason::
          kTemporalHealthDegraded;

  const auto json = RenderMapUpdateDecisionJson(snapshot, map_update_decision);

  EXPECT_NE(json.find("\"allowed\":false"), std::string::npos);
  EXPECT_NE(json.find("\"health\":\"DEGRADED\""), std::string::npos);
  EXPECT_NE(json.find("\"fault_reasons\":[\"lidar_stream_timing_jitter_high\"]"), std::string::npos);
  EXPECT_NE(json.find("reason=jitter_high"), std::string::npos);
  EXPECT_NE(json.find("window_max_jitter_ms=12.5"), std::string::npos);
}

TEST(MapUpdateDecisionJsonRendererTest, EscapesStrings) {
  causal_slam::diagnostics::TemporalDiagnosticSnapshot snapshot;
  snapshot.observation.has_lidar_scan = true;
  snapshot.observation.latest_lidar_frame_id = "lidar\\front\"";

  causal_slam::policy::MapUpdateDecision map_update_decision;
  map_update_decision.map_update_allowed = true;
  map_update_decision.reason =
      causal_slam::policy::MapUpdateDecisionReason::kTemporalHealthOk;

  const auto json = RenderMapUpdateDecisionJson(snapshot, map_update_decision);

  EXPECT_NE(json.find("lidar\\\\front\\\""), std::string::npos);
}

}  // namespace
}  // namespace causal_slam::render