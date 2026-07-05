#include "offline_temporal_report_document_builder.h"

#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace causal_slam::report {
namespace {

template <typename T>
std::string ToReportString(const T& value) {
  std::ostringstream out;
  out << value;
  return out.str();
}

ReportMetric Metric(std::string name, std::string value) {
  return ReportMetric{.name = std::move(name), .value = std::move(value)};
}

ReportSection MakeSection(std::string id, std::string title, std::string empty_message = "none") {
  ReportSection section;
  section.id = std::move(id);
  section.title = std::move(title);
  section.empty_message = std::move(empty_message);
  return section;
}

ReportRow MakeRow(std::string label, std::string status = {}, std::vector<ReportMetric> metrics = {}, std::string reason = {},
                  std::string detail = {}) {
  ReportRow row;
  row.label = std::move(label);
  row.status = std::move(status);
  row.metrics = std::move(metrics);
  row.reason = std::move(reason);
  row.detail = std::move(detail);
  return row;
}

std::string BoolString(bool value) {
  return value ? "true" : "false";
}

std::string NullableDoubleString(bool has_value, double value) {
  return has_value ? ToReportString(value) : "null";
}

ReportSection BuildVerdictSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  auto section = MakeSection("dataset_verdict", "Dataset verdict");

  section.metrics.push_back(Metric("health", report.verdict.health));
  section.metrics.push_back(Metric("reason", report.verdict.reason));
  section.metrics.push_back(Metric("fault_ratio", ToReportString(report.verdict.fault_ratio)));
  section.metrics.push_back(Metric("map_update_recommended", BoolString(report.verdict.map_update_recommended)));

  return section;
}

ReportSection BuildSelectedTopicsSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  auto section = MakeSection("selected_topics", "Selected topics");

  section.rows.push_back(MakeRow("lidar", report.lidar_topic_found ? "found" : "missing",
                                 {
                                     Metric("messages", ToReportString(report.lidar_messages)),
                                 }));

  section.rows.push_back(MakeRow("imu", report.imu_topic_found ? "found" : "missing",
                                 {
                                     Metric("messages", ToReportString(report.imu_messages)),
                                 }));

  return section;
}

ReportSection BuildTimingSection(std::string id, std::string title, const causal_slam::offline_analysis::TimingSummary& timing) {
  auto section = MakeSection(std::move(id), std::move(title));

  section.metrics.push_back(Metric("deserialized_messages", ToReportString(timing.deserialized_messages)));
  section.metrics.push_back(Metric("deserialization_failures", ToReportString(timing.deserialization_failures)));
  section.metrics.push_back(Metric("first_header_stamp_ns", ToReportString(timing.first_header_stamp_ns)));
  section.metrics.push_back(Metric("last_header_stamp_ns", ToReportString(timing.last_header_stamp_ns)));
  section.metrics.push_back(Metric("period_mean_ms", ToReportString(timing.period_mean_ms)));
  section.metrics.push_back(Metric("period_min_ms", ToReportString(timing.period_min_ms)));
  section.metrics.push_back(Metric("period_max_ms", ToReportString(timing.period_max_ms)));
  section.metrics.push_back(Metric("period_stddev_ms", ToReportString(timing.period_stddev_ms)));
  section.metrics.push_back(Metric("reorder_count", ToReportString(timing.reorder_count)));

  return section;
}

ReportSection BuildPointCloudCapabilitiesSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  const auto& caps = report.lidar_first_cloud.capabilities;
  auto section = MakeSection("pointcloud_capabilities", "PointCloud2 capabilities");

  section.metrics.push_back(Metric("has_xyz", BoolString(caps.has_x && caps.has_y && caps.has_z)));
  section.metrics.push_back(Metric("has_intensity", BoolString(caps.has_intensity)));
  section.metrics.push_back(Metric("has_ring", BoolString(caps.has_ring)));
  section.metrics.push_back(Metric("time_field_name", caps.time_field_name));
  section.metrics.push_back(Metric("time_field_datatype", caps.time_field_datatype));
  section.metrics.push_back(Metric("point_time_role", caps.point_time_role));
  section.metrics.push_back(Metric("point_time_supported", BoolString(caps.point_time_supported)));
  section.metrics.push_back(Metric("reason", caps.reason));

  return section;
}

ReportSection BuildLidarScanWindowsSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  const auto& windows = report.lidar_scan_windows;
  auto section = MakeSection("lidar_scan_windows", "LiDAR scan windows");

  section.metrics.push_back(Metric("scans_total", ToReportString(windows.scans_total)));
  section.metrics.push_back(Metric("estimated", ToReportString(windows.estimated)));
  section.metrics.push_back(Metric("with_point_time", ToReportString(windows.with_point_time)));
  section.metrics.push_back(Metric("failed", ToReportString(windows.failed)));
  section.metrics.push_back(Metric("duration_mean_ms", NullableDoubleString(windows.has_duration, windows.duration_mean_ms)));
  section.metrics.push_back(Metric("duration_min_ms", NullableDoubleString(windows.has_duration, windows.duration_min_ms)));
  section.metrics.push_back(Metric("duration_max_ms", NullableDoubleString(windows.has_duration, windows.duration_max_ms)));
  if (windows.duration_outlier_count > 0) {
    section.metrics.push_back(Metric("duration_outlier_count", ToReportString(windows.duration_outlier_count)));
    section.metrics.push_back(Metric("duration_outlier_ratio", ToReportString(windows.duration_outlier_ratio)));
    section.metrics.push_back(Metric("duration_outlier_threshold_ms", ToReportString(windows.duration_outlier_threshold_ms)));
    section.metrics.push_back(Metric("worst_duration_outlier_scan_index", ToReportString(windows.worst_duration_outlier_scan_index)));
    section.metrics.push_back(Metric("worst_duration_outlier_ms", ToReportString(windows.worst_duration_outlier_ms)));
  }
  section.metrics.push_back(Metric("source", windows.source));
  section.metrics.push_back(Metric("confidence", windows.confidence));
  section.metrics.push_back(Metric("time_unit", windows.time_unit));
  section.metrics.push_back(Metric("last_reason", windows.last_reason));

  return section;
}

ReportSection BuildImuCoverageSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  const auto& coverage = report.imu_coverage;
  auto section = MakeSection("imu_coverage", "Offline IMU coverage");

  section.metrics.push_back(Metric("scans_total", ToReportString(coverage.scans_total)));
  section.metrics.push_back(Metric("ok", ToReportString(coverage.ok)));
  section.metrics.push_back(Metric("warning", ToReportString(coverage.warning)));
  section.metrics.push_back(Metric("degraded", ToReportString(coverage.degraded)));
  section.metrics.push_back(Metric("invalid", ToReportString(coverage.invalid)));
  section.metrics.push_back(Metric("missing_prefix_count", ToReportString(coverage.missing_prefix_count)));
  section.metrics.push_back(Metric("missing_suffix_count", ToReportString(coverage.missing_suffix_count)));
  section.metrics.push_back(Metric("internal_gap_count", ToReportString(coverage.internal_gap_count)));
  section.metrics.push_back(Metric("mean_imu_count_in_window", ToReportString(coverage.mean_imu_count_in_window)));
  section.metrics.push_back(Metric("min_coverage_ratio", ToReportString(coverage.min_coverage_ratio)));
  section.metrics.push_back(Metric("mean_coverage_ratio", ToReportString(coverage.mean_coverage_ratio)));
  section.metrics.push_back(Metric("worst_reason", coverage.worst_reason));

  return section;
}

ReportSection BuildWorstSampleSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  const auto& coverage = report.imu_coverage;
  auto section = MakeSection("worst_imu_coverage_sample", "Worst IMU coverage sample");

  section.metrics.push_back(Metric("available", BoolString(coverage.has_worst_sample)));
  section.metrics.push_back(Metric("scan_index", ToReportString(coverage.worst_scan_index)));
  section.metrics.push_back(Metric("scan_start_ns", ToReportString(coverage.worst_scan_start_ns)));
  section.metrics.push_back(Metric("scan_end_ns", ToReportString(coverage.worst_scan_end_ns)));
  section.metrics.push_back(Metric("has_imu_bounds", BoolString(coverage.worst_has_imu_bounds)));
  section.metrics.push_back(Metric("first_imu_in_window_ns", ToReportString(coverage.worst_first_imu_in_window_ns)));
  section.metrics.push_back(Metric("last_imu_in_window_ns", ToReportString(coverage.worst_last_imu_in_window_ns)));
  section.metrics.push_back(Metric("reason", coverage.worst_sample_reason));
  section.metrics.push_back(Metric("imu_count_in_window", ToReportString(coverage.worst_imu_count_in_window)));
  section.metrics.push_back(Metric("missing_prefix_ms", ToReportString(coverage.worst_missing_prefix_ms)));
  section.metrics.push_back(Metric("missing_suffix_ms", ToReportString(coverage.worst_missing_suffix_ms)));
  section.metrics.push_back(Metric("max_gap_inside_ms", ToReportString(coverage.worst_max_gap_inside_ms)));
  section.metrics.push_back(Metric("coverage_ratio", ToReportString(coverage.worst_coverage_ratio)));

  return section;
}

ReportSection BuildFaultReasonsSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  auto section = MakeSection("fault_reasons", "Fault reasons", "no faults");

  for (const auto& [reason, count] : report.imu_coverage.fault_reasons) {
    section.rows.push_back(MakeRow(reason, "fault",
                                   {
                                       Metric("count", ToReportString(count)),
                                   }));
  }

  return section;
}

ReportSection BuildTopicsSection(const causal_slam::offline_analysis::OfflineTemporalReport& report) {
  auto section = MakeSection("topics", "Bag topics", "no topics");

  for (const auto& topic : report.topics) {
    section.rows.push_back(MakeRow(topic.name, topic.type,
                                   {
                                       Metric("messages", ToReportString(topic.message_count)),
                                   }));
  }

  return section;
}

}  // namespace

ReportDocument OfflineTemporalReportDocumentBuilder::Build(const causal_slam::offline_analysis::OfflineTemporalReport& report) const {
  ReportDocument document;
  document.title = "Offline temporal report";

  document.sections.push_back(BuildVerdictSection(report));
  document.sections.push_back(BuildSelectedTopicsSection(report));
  document.sections.push_back(BuildTimingSection("imu_timing", "IMU timing", report.imu_timing));
  document.sections.push_back(BuildTimingSection("lidar_timing", "LiDAR timing", report.lidar_timing));
  document.sections.push_back(BuildPointCloudCapabilitiesSection(report));
  document.sections.push_back(BuildLidarScanWindowsSection(report));
  document.sections.push_back(BuildImuCoverageSection(report));
  document.sections.push_back(BuildWorstSampleSection(report));
  document.sections.push_back(BuildFaultReasonsSection(report));
  document.sections.push_back(BuildTopicsSection(report));

  return document;
}

}  // namespace causal_slam::report
