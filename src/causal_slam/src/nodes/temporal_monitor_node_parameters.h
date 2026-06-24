#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "lidar/lidar_scan_window_estimator.h"
#include "pipeline/temporal_monitor_pipeline.h"

namespace causal_slam::nodes {

enum class RuntimeProfile : std::uint8_t {
  kMinimal,
  kDiagnostic,
  kDebugReport,
};

[[nodiscard]] const char* ToString(RuntimeProfile profile);

struct TransformCheckConfig {
  std::string target_frame;
  std::string source_frame;
};

struct TemporalMonitorNodeParameters {
  std::string imu_topic;
  std::string lidar_topic;
  std::string checked_lidar_topic;

  std::string map_update_allowed_topic;
  std::string temporal_health_topic;
  std::string map_update_reason_topic;
  std::string fault_reasons_topic;
  std::string map_update_decision_json_topic;

  std::string html_report_path;
  std::string lidar_gate_mode{"observe"};
  RuntimeProfile runtime_profile{RuntimeProfile::kDebugReport};

  double summary_period_ms{2000.0};

  double imu_gap_threshold_ms{100.0};
  double lidar_gap_threshold_ms{500.0};
  double lidar_scan_duration_ms{100.0};
  double lidar_min_measured_scan_duration_ms{1.0};
  double lidar_max_measured_scan_duration_ms{500.0};
  bool lidar_prefer_measured_header_period{true};
  causal_slam::lidar::LidarStampPolicy lidar_stamp_policy{causal_slam::lidar::LidarStampPolicy::kScanEnd};

  double expected_imu_period_ms{5.0};
  double imu_buffer_retention_ms{5000.0};
  double max_missing_prefix_ms{10.0};
  double max_missing_suffix_ms{10.0};
  double max_internal_gap_ms{25.0};

  bool tf_monitoring_enabled{false};
  std::vector<TransformCheckConfig> transform_checks;
  double tf_max_transform_age_ms{50.0};
  double tf_max_future_tolerance_ms{1.0};

  std::string lidar_qos_reliability{"best_effort"};
  int lidar_qos_depth{5};
  std::string checked_lidar_qos_reliability{"best_effort"};
  int checked_lidar_qos_depth{5};

  causal_slam::pipeline::TemporalMonitorPipelineConfig pipeline_config;
};

[[nodiscard]] TemporalMonitorNodeParameters LoadTemporalMonitorNodeParameters(rclcpp::Node& node);

}  // namespace causal_slam::nodes
