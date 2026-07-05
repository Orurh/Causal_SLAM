#include "presentation/render/offline_temporal_report_json_renderer.h"

#include <string>

#include <gtest/gtest.h>

namespace causal_slam::render {
namespace {

using causal_slam::offline_analysis::OfflineTemporalReport;

TEST(OfflineTemporalReportJsonRendererTest, RendersTopLevelSchemaAndVerdict) {
  OfflineTemporalReport report;
  report.lidar_messages = 10;
  report.imu_messages = 100;
  report.lidar_topic_found = true;
  report.imu_topic_found = true;

  report.verdict.health = "WARNING";
  report.verdict.reason = "isolated_imu_coverage_faults";
  report.verdict.fault_ratio = 0.25;
  report.verdict.map_update_recommended = true;

  report.imu_coverage.scans_total = 4;
  report.imu_coverage.ok = 3;
  report.imu_coverage.degraded = 1;
  report.imu_coverage.internal_gap_count = 2;
  report.imu_coverage.fault_reasons["imu_window_missing_prefix"] = 1;

  const OfflineTemporalReportRenderContext context{
      .bag_path = "/tmp/test_bag",
      .lidar_topic = "/points",
      .imu_topic = "/imu",
  };

  const OfflineTemporalReportJsonRenderer renderer;
  const std::string json = renderer.Render(context, report);

  EXPECT_NE(json.find("\"tool\": \"causal_slam_analyze_bag\""), std::string::npos);
  EXPECT_NE(json.find("\"schema_version\": 1"), std::string::npos);
  EXPECT_NE(json.find("\"bag\": \"/tmp/test_bag\""), std::string::npos);

  EXPECT_NE(json.find("\"selected_topics\""), std::string::npos);
  EXPECT_NE(json.find("\"lidar\": \"/points\""), std::string::npos);
  EXPECT_NE(json.find("\"imu\": \"/imu\""), std::string::npos);

  EXPECT_NE(json.find("\"verdict\""), std::string::npos);
  EXPECT_NE(json.find("\"health\": \"WARNING\""), std::string::npos);
  EXPECT_NE(json.find("\"reason\": \"isolated_imu_coverage_faults\""), std::string::npos);
  EXPECT_NE(json.find("\"map_update_recommended\": true"), std::string::npos);

  EXPECT_NE(json.find("\"imu_coverage\""), std::string::npos);
  EXPECT_NE(json.find("\"internal_gap_count\": 2"), std::string::npos);

  EXPECT_NE(json.find("\"fault_reasons\""), std::string::npos);
  EXPECT_NE(json.find("\"imu_window_missing_prefix\": 1"), std::string::npos);

  EXPECT_EQ(json.find("\"gap_count\""), std::string::npos);
}

TEST(OfflineTemporalReportJsonRendererTest, EscapesContextStrings) {
  OfflineTemporalReport report;

  const OfflineTemporalReportRenderContext context{
      .bag_path = "/tmp/bag \"quoted\"",
      .lidar_topic = "/points\\escaped",
      .imu_topic = "/imu\nnewline",
  };

  const OfflineTemporalReportJsonRenderer renderer;
  const std::string json = renderer.Render(context, report);

  EXPECT_NE(json.find("/tmp/bag \\\"quoted\\\""), std::string::npos);
  EXPECT_NE(json.find("/points\\\\escaped"), std::string::npos);
  EXPECT_NE(json.find("/imu\\nnewline"), std::string::npos);
}

}  // namespace
}  // namespace causal_slam::render
