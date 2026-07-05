#include "presentation/render/offline_temporal_report_console_renderer.h"

#include <string>

#include <gtest/gtest.h>

namespace causal_slam::render {
namespace {

using causal_slam::offline_analysis::OfflineTemporalReport;

TEST(OfflineTemporalReportConsoleRendererTest, RendersVerdictAndCoverageSummary) {
  OfflineTemporalReport report;
  report.lidar_messages = 2379;
  report.imu_messages = 44398;
  report.lidar_topic_found = true;
  report.imu_topic_found = true;

  report.verdict.health = "DEGRADED";
  report.verdict.reason = "imu_coverage_degraded";
  report.verdict.fault_ratio = 0.336696;
  report.verdict.map_update_recommended = false;

  report.imu_timing.deserialized_messages = 44398;
  report.imu_timing.deserialization_failures = 0;
  report.imu_timing.has_first_stamp = true;
  report.imu_timing.has_last_stamp = true;
  report.imu_timing.first_header_stamp_ns = 100;
  report.imu_timing.last_header_stamp_ns = 200;
  report.imu_timing.period_mean_ms = 3.69;
  report.imu_timing.period_min_ms = 0.58;
  report.imu_timing.period_max_ms = 102.2;
  report.imu_timing.period_stddev_ms = 5.84;
  report.imu_timing.reorder_count = 0;

  report.lidar_scan_windows.scans_total = 2379;
  report.lidar_scan_windows.estimated = 2379;
  report.lidar_scan_windows.with_point_time = 0;
  report.lidar_scan_windows.failed = 0;
  report.lidar_scan_windows.has_duration = true;
  report.lidar_scan_windows.duration_mean_ms = 68.95;
  report.lidar_scan_windows.duration_min_ms = 54.09;
  report.lidar_scan_windows.duration_max_ms = 200.03;
  report.lidar_scan_windows.source = "measured_header_period";
  report.lidar_scan_windows.confidence = "MEDIUM";
  report.lidar_scan_windows.time_unit = "none";
  report.lidar_scan_windows.last_reason = "measured_header_period";

  report.imu_coverage.scans_total = 2379;
  report.imu_coverage.ok = 1578;
  report.imu_coverage.degraded = 801;
  report.imu_coverage.missing_prefix_count = 379;
  report.imu_coverage.missing_suffix_count = 387;
  report.imu_coverage.internal_gap_count = 124;
  report.imu_coverage.mean_imu_count_in_window = 18.64;
  report.imu_coverage.min_coverage_ratio = 0.0;
  report.imu_coverage.mean_coverage_ratio = 0.807;
  report.imu_coverage.worst_reason = "imu_window_empty";
  report.imu_coverage.fault_reasons["imu_window_empty"] = 5;

  const OfflineTemporalReportConsoleRenderContext context{
      .bag_path = "/tmp/hard_bag",
      .report_path = "/tmp/report.json",
      .lidar_topic = "/points2/decompressed",
      .imu_topic = "/imu",
  };

  const OfflineTemporalReportConsoleRenderer renderer;
  const std::string text = renderer.Render(context, report);

  EXPECT_NE(text.find("Bag analyzed successfully."), std::string::npos);
  EXPECT_NE(text.find("bag: /tmp/hard_bag"), std::string::npos);
  EXPECT_NE(text.find("report: /tmp/report.json"), std::string::npos);

  EXPECT_NE(text.find("health=DEGRADED"), std::string::npos);
  EXPECT_NE(text.find("reason=imu_coverage_degraded"), std::string::npos);
  EXPECT_NE(text.find("map_update_recommended=false"), std::string::npos);

  EXPECT_NE(text.find("source=measured_header_period"), std::string::npos);
  EXPECT_NE(text.find("confidence=MEDIUM"), std::string::npos);
  EXPECT_NE(text.find("internal_gap_count=124"), std::string::npos);
  EXPECT_NE(text.find("imu_window_empty=5"), std::string::npos);

  EXPECT_EQ(text.find("\n  gap_count="), std::string::npos);
  EXPECT_EQ(text.find("\n    gap_count="), std::string::npos);
}

}  // namespace
}  // namespace causal_slam::render
