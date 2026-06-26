#pragma once

#include <string>
#include <vector>

#include "application/temporal_monitor/temporal_monitor_pipeline.h"

namespace causal_slam::config {

struct TemporalMonitorRuntimeLimits {
  double min_summary_period_ms{100.0};
  double min_gap_threshold_ms{1.0};
  double min_lidar_scan_duration_ms{1.0};
  double min_lidar_measured_scan_duration_ms{0.1};
  double min_imu_buffer_retention_ms{100.0};
  double min_expected_imu_period_ms{0.1};

  double imu_coverage_missing_prefix_periods{2.0};
  double imu_coverage_missing_suffix_periods{2.0};
  double imu_coverage_max_internal_gap_periods{5.0};
};

struct TemporalMonitorRuntimeDefaults {
  std::string imu_topic{"/imu/data"};
  std::string lidar_topic{"/points"};

  // Empty path disables LiDAR proxy/gate output.
  std::string checked_lidar_topic;
  std::string lidar_gate_mode{"observe"};

  std::string map_update_allowed_topic{"/causal_slam/map_update_allowed"};
  std::string temporal_health_topic{"/causal_slam/temporal_health"};
  std::string map_update_reason_topic{"/causal_slam/map_update_reason"};
  std::string fault_reasons_topic{"/causal_slam/fault_reasons"};
  std::string map_update_decision_json_topic{"/causal_slam/map_update_decision_json"};

  // Empty path disables periodic HTML report writing.
  std::string html_report_path;

  double summary_period_ms{2000.0};
  double expected_imu_period_ms{5.0};
  double imu_buffer_retention_ms{5000.0};

  std::uint64_t gate_min_total_imu_samples_before_forward{};
  std::uint64_t gate_min_window_imu_samples_before_forward{};

  bool tf_monitoring_enabled{false};
  std::vector<std::string> tf_target_frames{"odom"};
  std::vector<std::string> tf_source_frames{"<cloud_frame>"};

  causal_slam::pipeline::TemporalMonitorPipelineConfig pipeline;
  TemporalMonitorRuntimeLimits limits;
};

[[nodiscard]] TemporalMonitorRuntimeDefaults
MakeDefaultTemporalMonitorRuntimeDefaults();

}  // namespace causal_slam::config
