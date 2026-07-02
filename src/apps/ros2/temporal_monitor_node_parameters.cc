#include "temporal_monitor_node_parameters.h"

#include "domain/time/time_units.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "application/temporal_monitor/temporal_monitor_runtime_defaults.h"
#include "domain/sensors/imu/imu_coverage_analyzer.h"
#include "domain/sensors/lidar/lidar_scan_window_estimator.h"

namespace causal_slam::nodes {
namespace {

namespace config = causal_slam::config;
namespace coverage = causal_slam::coverage;
namespace lidar = causal_slam::lidar;
namespace pipeline = causal_slam::pipeline;

RuntimeProfile ParseRuntimeProfile(std::string_view value) {
  if (value == "minimal") {
    return RuntimeProfile::kMinimal;
  }

  if (value == "diagnostic") {
    return RuntimeProfile::kDiagnostic;
  }

  if (value == "debug_report") {
    return RuntimeProfile::kDebugReport;
  }

  // Preserve old verbose behavior for unknown values.
  return RuntimeProfile::kDebugReport;
}

lidar::LidarStampPolicy ParseLidarStampPolicy(std::string_view value) {
  if (value == "scan_start") {
    return lidar::LidarStampPolicy::kScanStart;
  }

  if (value == "scan_middle") {
    return lidar::LidarStampPolicy::kScanMiddle;
  }

  if (value == "scan_end") {
    return lidar::LidarStampPolicy::kScanEnd;
  }

  return lidar::LidarStampPolicy::kScanEnd;
}

std::vector<TransformCheckConfig> BuildTransformChecks(const std::vector<std::string>& target_frames,
                                                       const std::vector<std::string>& source_frames) {
  std::vector<TransformCheckConfig> checks;

  if (target_frames.size() != source_frames.size()) {
    return checks;
  }

  checks.reserve(target_frames.size());

  for (std::size_t i = 0; i < target_frames.size(); ++i) {
    checks.push_back(TransformCheckConfig{
        .target_frame = target_frames[i],
        .source_frame = source_frames[i],
    });
  }

  return checks;
}

}  // namespace

const char* ToString(RuntimeProfile profile) {
  switch (profile) {
    case RuntimeProfile::kMinimal:
      return "minimal";
    case RuntimeProfile::kDiagnostic:
      return "diagnostic";
    case RuntimeProfile::kDebugReport:
      return "debug_report";
  }

  return "debug_report";
}

pointcloud::PointCloud2TimeFieldOverrideMode ParsePointTimeMode(const std::string& value) {
  if (value == "explicit") {
    return pointcloud::PointCloud2TimeFieldOverrideMode::kExplicit;
  }

  if (value == "disabled") {
    return pointcloud::PointCloud2TimeFieldOverrideMode::kDisabled;
  }

  return pointcloud::PointCloud2TimeFieldOverrideMode::kAuto;
}

pointcloud::PointCloud2TimeFieldOverrideInterpretation ParsePointTimeInterpretation(const std::string& value) {
  if (value == "absolute") {
    return pointcloud::PointCloud2TimeFieldOverrideInterpretation::kAbsolute;
  }

  if (value == "relative") {
    return pointcloud::PointCloud2TimeFieldOverrideInterpretation::kRelative;
  }

  return pointcloud::PointCloud2TimeFieldOverrideInterpretation::kAuto;
}

pointcloud::PointCloud2TimeFieldOverrideUnit ParsePointTimeUnit(const std::string& value) {
  if (value == "seconds") {
    return pointcloud::PointCloud2TimeFieldOverrideUnit::kSeconds;
  }

  if (value == "nanoseconds") {
    return pointcloud::PointCloud2TimeFieldOverrideUnit::kNanoseconds;
  }

  return pointcloud::PointCloud2TimeFieldOverrideUnit::kAuto;
}

TemporalMonitorNodeParameters LoadTemporalMonitorNodeParameters(rclcpp::Node& node) {
  const auto defaults = config::MakeDefaultTemporalMonitorRuntimeDefaults();

  TemporalMonitorNodeParameters params;

  params.runtime_profile = ParseRuntimeProfile(node.declare_parameter<std::string>("runtime_profile", "debug_report"));

  params.imu_topic = node.declare_parameter<std::string>("imu_topic", defaults.imu_topic);
  params.lidar_topic = node.declare_parameter<std::string>("lidar_topic", defaults.lidar_topic);
  params.checked_lidar_topic = node.declare_parameter<std::string>("checked_lidar_topic", defaults.checked_lidar_topic);

  params.lidar_gate_mode = node.declare_parameter<std::string>("lidar_gate_mode", defaults.lidar_gate_mode);
  const auto gate_min_total_imu_samples_before_forward = node.declare_parameter<std::int64_t>(
      "gate_min_total_imu_samples_before_forward", static_cast<std::int64_t>(defaults.gate_min_total_imu_samples_before_forward));
  params.gate_min_total_imu_samples_before_forward =
      static_cast<std::uint64_t>(std::max<std::int64_t>(gate_min_total_imu_samples_before_forward, 0));

  const auto gate_min_window_imu_samples_before_forward = node.declare_parameter<std::int64_t>(
      "gate_min_window_imu_samples_before_forward", static_cast<std::int64_t>(defaults.gate_min_window_imu_samples_before_forward));
  params.gate_min_window_imu_samples_before_forward =
      static_cast<std::uint64_t>(std::max<std::int64_t>(gate_min_window_imu_samples_before_forward, 0));

  params.map_update_allowed_topic = node.declare_parameter<std::string>("map_update_allowed_topic", defaults.map_update_allowed_topic);
  params.temporal_health_topic = node.declare_parameter<std::string>("temporal_health_topic", defaults.temporal_health_topic);
  params.map_update_reason_topic = node.declare_parameter<std::string>("map_update_reason_topic", defaults.map_update_reason_topic);
  params.fault_reasons_topic = node.declare_parameter<std::string>("fault_reasons_topic", defaults.fault_reasons_topic);
  params.map_update_decision_json_topic =
      node.declare_parameter<std::string>("map_update_decision_json_topic", defaults.map_update_decision_json_topic);

  params.html_report_path = node.declare_parameter<std::string>("html_report_path", defaults.html_report_path);

  const double summary_period_ms = node.declare_parameter<double>("summary_period_ms", defaults.summary_period_ms);
  params.summary_period_ms = std::max(summary_period_ms, defaults.limits.min_summary_period_ms);

  const double imu_gap_threshold_ms = node.declare_parameter<double>("imu_gap_threshold_ms", defaults.pipeline.imu_gap_threshold_ms);
  params.imu_gap_threshold_ms = std::max(imu_gap_threshold_ms, defaults.limits.min_gap_threshold_ms);

  const double lidar_gap_threshold_ms = node.declare_parameter<double>("lidar_gap_threshold_ms", defaults.pipeline.lidar_gap_threshold_ms);
  params.lidar_gap_threshold_ms = std::max(lidar_gap_threshold_ms, defaults.limits.min_gap_threshold_ms);

  const double lidar_scan_duration_ms =
      node.declare_parameter<double>("lidar_scan_duration_ms", defaults.pipeline.lidar_scan_window.fallback_scan_duration_ms);
  params.lidar_scan_duration_ms = std::max(lidar_scan_duration_ms, defaults.limits.min_lidar_scan_duration_ms);

  const double lidar_min_measured_scan_duration_ms = node.declare_parameter<double>(
      "lidar_min_measured_scan_duration_ms", defaults.pipeline.lidar_scan_window.min_measured_scan_duration_ms);
  params.lidar_min_measured_scan_duration_ms =
      std::max(lidar_min_measured_scan_duration_ms, defaults.limits.min_lidar_measured_scan_duration_ms);

  const double lidar_max_measured_scan_duration_ms = node.declare_parameter<double>(
      "lidar_max_measured_scan_duration_ms", defaults.pipeline.lidar_scan_window.max_measured_scan_duration_ms);
  params.lidar_max_measured_scan_duration_ms = std::max(lidar_max_measured_scan_duration_ms, params.lidar_min_measured_scan_duration_ms);

  params.lidar_prefer_measured_header_period = node.declare_parameter<bool>(
      "lidar_prefer_measured_header_period", defaults.pipeline.lidar_scan_window.prefer_measured_header_period);

  const std::string lidar_stamp_policy = node.declare_parameter<std::string>(
      "lidar_stamp_policy", std::string{lidar::ToString(defaults.pipeline.lidar_scan_window.stamp_policy)});
  params.lidar_stamp_policy = ParseLidarStampPolicy(lidar_stamp_policy);

  params.lidar_point_time_mode = node.declare_parameter<std::string>("lidar_point_time_mode", "auto");
  params.lidar_point_time_field = node.declare_parameter<std::string>("lidar_point_time_field", "");
  params.lidar_point_time_interpretation = node.declare_parameter<std::string>("lidar_point_time_interpretation", "auto");
  params.lidar_point_time_unit = node.declare_parameter<std::string>("lidar_point_time_unit", "auto");

  params.lidar_holdback_enabled = node.declare_parameter<bool>("lidar_holdback_enabled", false);

  const double lidar_holdback_tolerance_ms = node.declare_parameter<double>("lidar_holdback_tolerance_ms", 10.0);
  params.lidar_holdback_tolerance_ms = std::max(lidar_holdback_tolerance_ms, 0.0);

  const auto lidar_holdback_max_pending = node.declare_parameter<int>("lidar_holdback_max_pending", 32);
  params.lidar_holdback_max_pending = static_cast<int>(std::max<std::int64_t>(lidar_holdback_max_pending, 1));
  const double imu_buffer_retention_ms = node.declare_parameter<double>("imu_buffer_retention_ms", defaults.imu_buffer_retention_ms);
  params.imu_buffer_retention_ms = std::max(imu_buffer_retention_ms, defaults.limits.min_imu_buffer_retention_ms);

  const double expected_imu_period_ms = node.declare_parameter<double>("expected_imu_period_ms", defaults.expected_imu_period_ms);
  params.expected_imu_period_ms = std::max(expected_imu_period_ms, defaults.limits.min_expected_imu_period_ms);

  const double max_missing_prefix_ms = node.declare_parameter<double>(
      "max_missing_prefix_ms", defaults.limits.imu_coverage_missing_prefix_periods * params.expected_imu_period_ms);
  const double max_missing_suffix_ms = node.declare_parameter<double>(
      "max_missing_suffix_ms", defaults.limits.imu_coverage_missing_suffix_periods * params.expected_imu_period_ms);
  const double max_internal_gap_ms = node.declare_parameter<double>(
      "max_internal_gap_ms", defaults.limits.imu_coverage_max_internal_gap_periods * params.expected_imu_period_ms);

  params.max_missing_prefix_ms = std::max(max_missing_prefix_ms, 0.0);
  params.max_missing_suffix_ms = std::max(max_missing_suffix_ms, 0.0);
  params.max_internal_gap_ms = std::max(max_internal_gap_ms, params.expected_imu_period_ms);

  params.tf_monitoring_enabled = node.declare_parameter<bool>("tf_monitoring_enabled", defaults.tf_monitoring_enabled);

  const std::string tf_target_frame = node.declare_parameter<std::string>("tf_target_frame", defaults.tf_target_frames.front());
  const std::string tf_source_frame = node.declare_parameter<std::string>("tf_source_frame", defaults.tf_source_frames.front());

  const std::vector<std::string> tf_target_frames =
      node.declare_parameter<std::vector<std::string>>("tf_target_frames", std::vector<std::string>{tf_target_frame});
  const std::vector<std::string> tf_source_frames =
      node.declare_parameter<std::vector<std::string>>("tf_source_frames", std::vector<std::string>{tf_source_frame});

  params.tf_max_transform_age_ms =
      std::max(node.declare_parameter<double>("tf_max_transform_age_ms", defaults.pipeline.transform_age.max_transform_age_ms), 0.0);
  params.tf_max_future_tolerance_ms =
      std::max(node.declare_parameter<double>("tf_max_future_tolerance_ms", defaults.pipeline.transform_age.max_future_tolerance_ms), 0.0);

  params.transform_checks = BuildTransformChecks(tf_target_frames, tf_source_frames);

  params.lidar_qos_reliability = node.declare_parameter<std::string>("lidar_qos_reliability", "best_effort");
  params.lidar_qos_depth = std::max(static_cast<int>(node.declare_parameter<int>("lidar_qos_depth", 5)), 1);

  params.checked_lidar_qos_reliability = node.declare_parameter<std::string>("checked_lidar_qos_reliability", params.lidar_qos_reliability);
  params.checked_lidar_qos_depth =
      std::max(static_cast<int>(node.declare_parameter<int>("checked_lidar_qos_depth", params.lidar_qos_depth)), 1);

  params.pipeline_config = pipeline::TemporalMonitorPipelineConfig{};
  params.pipeline_config.imu_gap_threshold_ms = params.imu_gap_threshold_ms;
  params.pipeline_config.lidar_gap_threshold_ms = params.lidar_gap_threshold_ms;
  params.pipeline_config.imu_buffer_retention_ns = core::MillisecondsToNanoseconds(std::max(params.imu_buffer_retention_ms, 0.0));
  params.pipeline_config.lidar_scan_window = lidar::LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = params.lidar_scan_duration_ms,
      .min_measured_scan_duration_ms = params.lidar_min_measured_scan_duration_ms,
      .max_measured_scan_duration_ms = params.lidar_max_measured_scan_duration_ms,
      .stamp_policy = params.lidar_stamp_policy,
      .prefer_measured_header_period = params.lidar_prefer_measured_header_period,
  };
  params.pipeline_config.point_time = pointcloud::PointCloud2TimeFieldOverrideConfig{
      .mode = ParsePointTimeMode(params.lidar_point_time_mode),
      .field_name = params.lidar_point_time_field,
      .interpretation = ParsePointTimeInterpretation(params.lidar_point_time_interpretation),
      .unit = ParsePointTimeUnit(params.lidar_point_time_unit),
  };
  params.pipeline_config.imu_coverage = coverage::ImuCoverageConfig{
      .max_missing_prefix_ms = params.max_missing_prefix_ms,
      .max_missing_suffix_ms = params.max_missing_suffix_ms,
      .max_internal_gap_ms = params.max_internal_gap_ms,
  };
  params.pipeline_config.transform_age.max_transform_age_ms = params.tf_max_transform_age_ms;
  params.pipeline_config.transform_age.max_future_tolerance_ms = params.tf_max_future_tolerance_ms;

  return params;
}

}  // namespace causal_slam::nodes
