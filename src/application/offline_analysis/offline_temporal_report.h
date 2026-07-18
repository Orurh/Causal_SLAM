#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace causal_slam::offline_analysis {

struct TopicSummary {
  std::string name;
  std::string type;
  std::uint64_t message_count = 0;
};

struct TimingSummary {
  std::uint64_t deserialized_messages = 0;
  std::uint64_t deserialization_failures = 0;
  bool has_first_stamp = false;
  bool has_last_stamp = false;
  std::int64_t first_header_stamp_ns = 0;
  std::int64_t last_header_stamp_ns = 0;
  bool has_period = false;
  std::uint64_t period_count = 0;
  double period_mean_ms = 0.0;
  double period_min_ms = 0.0;
  double period_max_ms = 0.0;
  double period_stddev_ms = 0.0;
  double jitter_stddev_ms = 0.0;
  std::uint64_t reorder_count = 0;
};

struct PointFieldSummary {
  std::string name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
  std::string datatype_name;
};

struct PointCloudCapabilitySummary {
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  bool has_ring = false;
  bool has_line = false;
  bool has_channel = false;
  bool has_time_field = false;
  std::string time_field_name;
  std::string time_field_datatype;
  std::string point_time_role = "none";
  bool point_time_supported = false;
  std::string reason = "no point time field";
};

struct LidarFirstCloudSummary {
  bool observed = false;
  std::string frame_id;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  std::size_t data_size = 0;
  std::size_t fields_count = 0;
  std::vector<PointFieldSummary> fields;
  PointCloudCapabilitySummary capabilities;
};

struct LidarScanWindowSummary {
  std::uint64_t scans_total = 0;
  std::uint64_t estimated = 0;
  std::uint64_t with_point_time = 0;
  std::uint64_t failed = 0;
  std::uint64_t points_total = 0;
  std::uint64_t points_used = 0;

  bool has_duration = false;
  double duration_mean_ms = 0.0;
  double duration_min_ms = 0.0;
  double duration_max_ms = 0.0;
  double duration_stddev_ms = 0.0;

  std::uint64_t duration_outlier_count = 0;
  double duration_outlier_ratio = 0.0;
  double duration_outlier_threshold_ms = 200.0;
  std::uint64_t worst_duration_outlier_scan_index = 0;
  double worst_duration_outlier_ms = 0.0;

  std::string source = "none";
  std::string confidence = "UNKNOWN";
  std::string time_unit = "unknown";
  std::string last_reason = "not_started";
};

struct OfflineImuCoverageReport {
  std::uint64_t scans_total = 0;
  std::uint64_t ok = 0;
  std::uint64_t warning = 0;
  std::uint64_t degraded = 0;
  std::uint64_t invalid = 0;
  std::uint64_t missing_prefix_count = 0;
  std::uint64_t missing_suffix_count = 0;
  std::uint64_t internal_gap_count = 0;

  bool has_observed_imu_period_p95 = false;
  double observed_imu_period_p95_ms = 0.0;
  double configured_min_edge_tolerance_ms = 0.0;
  double adaptive_edge_tolerance_ms = 0.0;
  double effective_edge_tolerance_ms = 0.0;
  std::string edge_tolerance_source = "configured_minimum";

  std::uint64_t min_imu_count_in_window = 0;
  std::uint64_t max_imu_count_in_window = 0;
  double mean_imu_count_in_window = 0.0;
  double min_coverage_ratio = 0.0;
  double mean_coverage_ratio = 0.0;
  std::string worst_reason = "not_analyzed";

  bool has_worst_sample = false;
  std::uint64_t worst_scan_index = 0;
  std::string worst_sample_reason = "none";
  std::int64_t worst_scan_start_ns = 0;
  std::int64_t worst_scan_end_ns = 0;
  bool worst_has_imu_bounds = false;
  std::int64_t worst_first_imu_in_window_ns = 0;
  std::int64_t worst_last_imu_in_window_ns = 0;
  std::uint64_t worst_imu_count_in_window = 0;
  double worst_missing_prefix_ms = 0.0;
  double worst_missing_suffix_ms = 0.0;
  double worst_max_gap_inside_ms = 0.0;
  double worst_coverage_ratio = 0.0;

  std::map<std::string, std::uint64_t> fault_reasons;
};

struct DatasetVerdict {
  std::string health = "UNKNOWN";
  std::string reason = "not_analyzed";
  double fault_ratio = 0.0;
  bool map_update_recommended = false;
};

struct StreamTimingFaultReport {
  bool lidar_stream_timing_jitter_high = false;
  bool lidar_stream_timing_short_period = false;
  bool lidar_stream_timing_long_period = false;

  double lidar_period_jitter_threshold_ms = 5.0;
  double lidar_period_short_threshold_ratio = 0.8;
  double lidar_period_long_threshold_ratio = 1.2;

  std::map<std::string, std::uint64_t> fault_reasons;
};

struct OfflineTemporalReport {
  std::vector<TopicSummary> topics;
  std::uint64_t lidar_messages = 0;
  std::uint64_t imu_messages = 0;
  bool lidar_topic_found = false;
  bool imu_topic_found = false;
  TimingSummary imu_timing;
  TimingSummary lidar_timing;
  LidarFirstCloudSummary lidar_first_cloud;
  LidarScanWindowSummary lidar_scan_windows;
  OfflineImuCoverageReport imu_coverage;
  StreamTimingFaultReport stream_timing_faults;
  DatasetVerdict verdict;
};

// Temporary compatibility name while analyze_bag_app.cc is being slimmed down.
using BagSummary = OfflineTemporalReport;

}  // namespace causal_slam::offline_analysis
