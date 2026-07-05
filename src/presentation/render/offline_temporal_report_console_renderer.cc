#include "offline_temporal_report_console_renderer.h"

#include <sstream>

namespace causal_slam::render {

std::string OfflineTemporalReportConsoleRenderer::Render(const OfflineTemporalReportConsoleRenderContext& context,
                                                         const causal_slam::offline_analysis::OfflineTemporalReport& summary) const {
  std::ostringstream out;

  out << "Bag analyzed successfully.\n";
  out << "  bag: " << context.bag_path << "\n";
  out << "  report: " << context.report_path << "\n\n";

  out << "Selected topics:\n";
  out << "  lidar: " << context.lidar_topic << " messages=" << summary.lidar_messages
      << " found=" << (summary.lidar_topic_found ? "true" : "false") << "\n";
  out << "  imu: " << context.imu_topic << " messages=" << summary.imu_messages << " found=" << (summary.imu_topic_found ? "true" : "false")
      << "\n\n";

  out << "Verdict:\n";
  out << "  health=" << summary.verdict.health << "\n";
  out << "  reason=" << summary.verdict.reason << "\n";
  out << "  fault_ratio=" << summary.verdict.fault_ratio << "\n";
  out << "  map_update_recommended=" << (summary.verdict.map_update_recommended ? "true" : "false") << "\n\n";

  out << "IMU timing:\n";
  out << "  deserialized_messages=" << summary.imu_timing.deserialized_messages << "\n";
  out << "  deserialization_failures=" << summary.imu_timing.deserialization_failures << "\n";
  out << "  first_header_stamp_ns=" << summary.imu_timing.first_header_stamp_ns << "\n";
  out << "  last_header_stamp_ns=" << summary.imu_timing.last_header_stamp_ns << "\n";
  out << "  period_mean_ms=" << summary.imu_timing.period_mean_ms << "\n";
  out << "  period_min_ms=" << summary.imu_timing.period_min_ms << "\n";
  out << "  period_max_ms=" << summary.imu_timing.period_max_ms << "\n";
  out << "  period_stddev_ms=" << summary.imu_timing.period_stddev_ms << "\n";
  out << "  reorder_count=" << summary.imu_timing.reorder_count << "\n";

  out << "LiDAR timing:\n";
  out << "  deserialized_messages=" << summary.lidar_timing.deserialized_messages << "\n";
  out << "  deserialization_failures=" << summary.lidar_timing.deserialization_failures << "\n";
  out << "  first_header_stamp_ns=" << summary.lidar_timing.first_header_stamp_ns << "\n";
  out << "  last_header_stamp_ns=" << summary.lidar_timing.last_header_stamp_ns << "\n";
  out << "  period_mean_ms=" << summary.lidar_timing.period_mean_ms << "\n";
  out << "  period_min_ms=" << summary.lidar_timing.period_min_ms << "\n";
  out << "  period_max_ms=" << summary.lidar_timing.period_max_ms << "\n";
  out << "  period_stddev_ms=" << summary.lidar_timing.period_stddev_ms << "\n";
  out << "  reorder_count=" << summary.lidar_timing.reorder_count << "\n";

  out << "PointCloud2 capabilities:\n";
  out << "  has_xyz="
      << ((summary.lidar_first_cloud.capabilities.has_x && summary.lidar_first_cloud.capabilities.has_y &&
           summary.lidar_first_cloud.capabilities.has_z)
              ? "true"
              : "false")
      << "\n";
  out << "  has_intensity=" << (summary.lidar_first_cloud.capabilities.has_intensity ? "true" : "false") << "\n";
  out << "  has_ring=" << (summary.lidar_first_cloud.capabilities.has_ring ? "true" : "false") << "\n";
  out << "  time_field_name=" << summary.lidar_first_cloud.capabilities.time_field_name << "\n";
  out << "  time_field_datatype=" << summary.lidar_first_cloud.capabilities.time_field_datatype << "\n";
  out << "  point_time_role=" << summary.lidar_first_cloud.capabilities.point_time_role << "\n";
  out << "  point_time_supported=" << (summary.lidar_first_cloud.capabilities.point_time_supported ? "true" : "false") << "\n";
  out << "  reason=" << summary.lidar_first_cloud.capabilities.reason << "\n\n";

  out << "LiDAR scan windows:\n";
  out << "  scans_total=" << summary.lidar_scan_windows.scans_total << "\n";
  out << "  estimated=" << summary.lidar_scan_windows.estimated << "\n";
  out << "  with_point_time=" << summary.lidar_scan_windows.with_point_time << "\n";
  out << "  failed=" << summary.lidar_scan_windows.failed << "\n";
  out << "  duration_mean_ms=" << summary.lidar_scan_windows.duration_mean_ms << "\n";
  out << "  duration_min_ms=" << summary.lidar_scan_windows.duration_min_ms << "\n";
  out << "  duration_max_ms=" << summary.lidar_scan_windows.duration_max_ms << "\n";
  if (summary.lidar_scan_windows.duration_outlier_count > 0) {
    out << "  duration_outlier_count=" << summary.lidar_scan_windows.duration_outlier_count << "\n";
    out << "  duration_outlier_ratio=" << summary.lidar_scan_windows.duration_outlier_ratio << "\n";
    out << "  duration_outlier_threshold_ms=" << summary.lidar_scan_windows.duration_outlier_threshold_ms << "\n";
    out << "  worst_duration_outlier_scan_index=" << summary.lidar_scan_windows.worst_duration_outlier_scan_index << "\n";
    out << "  worst_duration_outlier_ms=" << summary.lidar_scan_windows.worst_duration_outlier_ms << "\n";
  }
  out << "  source=" << summary.lidar_scan_windows.source << "\n";
  out << "  confidence=" << summary.lidar_scan_windows.confidence << "\n";
  out << "  time_unit=" << summary.lidar_scan_windows.time_unit << "\n";
  out << "  last_reason=" << summary.lidar_scan_windows.last_reason << "\n\n";

  out << "Offline IMU coverage:\n";
  out << "  scans_total=" << summary.imu_coverage.scans_total << "\n";
  out << "  ok=" << summary.imu_coverage.ok << "\n";
  out << "  warning=" << summary.imu_coverage.warning << "\n";
  out << "  degraded=" << summary.imu_coverage.degraded << "\n";
  out << "  invalid=" << summary.imu_coverage.invalid << "\n";
  out << "  missing_prefix_count=" << summary.imu_coverage.missing_prefix_count << "\n";
  out << "  missing_suffix_count=" << summary.imu_coverage.missing_suffix_count << "\n";
  out << "  internal_gap_count=" << summary.imu_coverage.internal_gap_count << "\n";
  out << "  mean_imu_count_in_window=" << summary.imu_coverage.mean_imu_count_in_window << "\n";
  out << "  min_coverage_ratio=" << summary.imu_coverage.min_coverage_ratio << "\n";
  out << "  mean_coverage_ratio=" << summary.imu_coverage.mean_coverage_ratio << "\n";
  out << "  worst_reason=" << summary.imu_coverage.worst_reason << "\n\n";

  out << "Worst IMU coverage sample:\n";
  if (!summary.imu_coverage.has_worst_sample) {
    out << "  none\n\n";
  } else {
    out << "  scan_index=" << summary.imu_coverage.worst_scan_index << "\n";
    out << "  scan_start_ns=" << summary.imu_coverage.worst_scan_start_ns << "\n";
    out << "  scan_end_ns=" << summary.imu_coverage.worst_scan_end_ns << "\n";
    out << "  has_imu_bounds=" << (summary.imu_coverage.worst_has_imu_bounds ? "true" : "false") << "\n";
    out << "  first_imu_in_window_ns=" << summary.imu_coverage.worst_first_imu_in_window_ns << "\n";
    out << "  last_imu_in_window_ns=" << summary.imu_coverage.worst_last_imu_in_window_ns << "\n";
    out << "  reason=" << summary.imu_coverage.worst_sample_reason << "\n";
    out << "  imu_count_in_window=" << summary.imu_coverage.worst_imu_count_in_window << "\n";
    out << "  missing_prefix_ms=" << summary.imu_coverage.worst_missing_prefix_ms << "\n";
    out << "  missing_suffix_ms=" << summary.imu_coverage.worst_missing_suffix_ms << "\n";
    out << "  max_gap_inside_ms=" << summary.imu_coverage.worst_max_gap_inside_ms << "\n";
    out << "  coverage_ratio=" << summary.imu_coverage.worst_coverage_ratio << "\n\n";
  }

  out << "Fault reasons:\n";
  if (summary.imu_coverage.fault_reasons.empty()) {
    out << "  none\n";
  } else {
    for (const auto& [reason, count] : summary.imu_coverage.fault_reasons) {
      out << "  " << reason << "=" << count << "\n";
    }
  }
  out << "\n";

  out << "Topics:\n";
  for (const auto& topic : summary.topics) {
    out << "  " << topic.name << " " << topic.type << " messages=" << topic.message_count << "\n";
  }

  return out.str();
}

}  // namespace causal_slam::render
