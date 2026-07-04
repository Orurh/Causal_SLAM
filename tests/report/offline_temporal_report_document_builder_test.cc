#include "presentation/report/offline_temporal_report_document_builder.h"

#include <algorithm>
#include <string_view>

#include <gtest/gtest.h>

namespace causal_slam::report {
namespace {

const ReportSection* FindSection(const ReportDocument& document, std::string_view id) {
  const auto it = std::find_if(document.sections.begin(), document.sections.end(),
                               [id](const ReportSection& section) { return section.id == id; });
  if (it == document.sections.end()) {
    return nullptr;
  }
  return &(*it);
}

const ReportMetric* FindMetric(const ReportSection& section, std::string_view name) {
  const auto it = std::find_if(section.metrics.begin(), section.metrics.end(),
                               [name](const ReportMetric& metric) { return metric.name == name; });
  if (it == section.metrics.end()) {
    return nullptr;
  }
  return &(*it);
}

TEST(OfflineTemporalReportDocumentBuilderTest, BuildsCoreSections) {
  causal_slam::offline_analysis::OfflineTemporalReport report;
  report.verdict.health = "DEGRADED";
  report.verdict.reason = "imu_coverage_degraded";
  report.verdict.map_update_recommended = false;

  report.lidar_messages = 2379;
  report.imu_messages = 44398;
  report.lidar_topic_found = true;
  report.imu_topic_found = true;

  report.imu_coverage.scans_total = 2379;
  report.imu_coverage.degraded = 801;
  report.imu_coverage.internal_gap_count = 124;
  report.imu_coverage.worst_reason = "imu_window_empty";
  report.imu_coverage.fault_reasons["imu_window_empty"] = 5;

  report.lidar_scan_windows.scans_total = 2379;
  report.lidar_scan_windows.estimated = 2379;
  report.lidar_scan_windows.source = "measured_header_period";
  report.lidar_scan_windows.confidence = "MEDIUM";
  report.lidar_scan_windows.time_unit = "none";

  report.topics.push_back(causal_slam::offline_analysis::TopicSummary{
      .name = "/imu",
      .type = "sensor_msgs/msg/Imu",
      .message_count = 44398,
  });

  const OfflineTemporalReportDocumentBuilder builder;
  const ReportDocument document = builder.Build(report);

  EXPECT_EQ(document.title, "Offline temporal report");

  const ReportSection* verdict = FindSection(document, "dataset_verdict");
  ASSERT_NE(verdict, nullptr);
  ASSERT_NE(FindMetric(*verdict, "health"), nullptr);
  EXPECT_EQ(FindMetric(*verdict, "health")->value, "DEGRADED");
  ASSERT_NE(FindMetric(*verdict, "reason"), nullptr);
  EXPECT_EQ(FindMetric(*verdict, "reason")->value, "imu_coverage_degraded");

  const ReportSection* coverage = FindSection(document, "imu_coverage");
  ASSERT_NE(coverage, nullptr);
  ASSERT_NE(FindMetric(*coverage, "internal_gap_count"), nullptr);
  EXPECT_EQ(FindMetric(*coverage, "internal_gap_count")->value, "124");

  const ReportSection* faults = FindSection(document, "fault_reasons");
  ASSERT_NE(faults, nullptr);
  ASSERT_EQ(faults->rows.size(), 1U);
  EXPECT_EQ(faults->rows[0].label, "imu_window_empty");

  const ReportSection* topics = FindSection(document, "topics");
  ASSERT_NE(topics, nullptr);
  ASSERT_EQ(topics->rows.size(), 1U);
  EXPECT_EQ(topics->rows[0].label, "/imu");
}

}  // namespace
}  // namespace causal_slam::report
