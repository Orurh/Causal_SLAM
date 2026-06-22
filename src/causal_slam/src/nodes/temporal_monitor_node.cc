#include "temporal_monitor_node.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "coverage/imu_coverage_analyzer.h"
#include "diagnostics/temporal_fault_reason_formatter.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "model/temporal_observation.h"
#include "policy/map_update_decision.h"
#include "render/console_temporal_summary_renderer.h"
#include "ros_adapters/point_cloud2_conversions.h"

namespace causal_slam::nodes {

namespace coverage = causal_slam::coverage;
namespace diagnostics = causal_slam::diagnostics;
namespace lidar = causal_slam::lidar;
namespace pipeline = causal_slam::pipeline;
namespace render = causal_slam::render;
namespace ros_adapters = causal_slam::ros_adapters;
namespace statistics = causal_slam::statistics;
namespace telemetry = causal_slam::telemetry;

namespace {

void LogTimingSummary(
    const rclcpp::Logger& logger,
    const telemetry::StreamTimingDiagnostic& stream) {
  const auto& summary = stream.timing;

  RCLCPP_INFO_STREAM(
      logger, telemetry::ToString(stream.id)
                  << " timing"
                  << " | total_count=" << summary.total_count
                  << " | window_count=" << summary.window_count
                  << " | last_delay_ms=" << summary.last_delay_ms
                  << " | window_avg_delay_ms="
                  << summary.window_average_delay_ms
                  << " | window_max_delay_ms=" << summary.window_max_delay_ms
                  << " | last_period_ms=" << summary.last_period_ms
                  << " | last_jitter_ms=" << summary.last_jitter_ms
                  << " | window_max_jitter_ms="
                  << summary.window_max_jitter_ms
                  << " | total_reordered_count="
                  << summary.total_reordered_count
                  << " | window_reordered_count="
                  << summary.window_reordered_count
                  << " | total_gap_count=" << summary.total_gap_count
                  << " | window_gap_count=" << summary.window_gap_count
                  << " | max_gap_ms=" << summary.max_gap_ms
                  << " | health=" << telemetry::ToString(summary.health)
                  << " | reason=" << summary.reason);
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

std::int64_t MillisecondsToNanoseconds(double milliseconds) {
  constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

  const double safe_milliseconds = std::max(milliseconds, 0.0);
  return static_cast<std::int64_t>(
      safe_milliseconds * kNanosecondsPerMillisecond);
}

void LogImuCoverageSummary(
    const rclcpp::Logger& logger,
    const std::optional<coverage::ImuCoverageSummary>& summary,
    const std::optional<lidar::LidarScanWindowEstimate>& scan_window_estimate,
    std::size_t imu_buffer_size) {
  if (!summary.has_value() || !scan_window_estimate.has_value()) {
    RCLCPP_INFO_STREAM(logger, "IMU coverage"
                                   << " | status=not_available"
                                   << " | reason=no_lidar_scan_received_yet"
                                   << " | imu_buffer_size=" << imu_buffer_size);
    return;
  }

  const auto& coverage_summary = *summary;
  const auto& scan_window = *scan_window_estimate;

  RCLCPP_INFO_STREAM(
      logger, "IMU coverage"
                  << " | imu_count_in_window="
                  << coverage_summary.imu_count_in_window
                  << " | missing_prefix_ms="
                  << coverage_summary.missing_prefix_ms
                  << " | missing_suffix_ms="
                  << coverage_summary.missing_suffix_ms
                  << " | max_gap_inside_ms="
                  << coverage_summary.max_gap_inside_ms
                  << " | coverage_ratio=" << coverage_summary.coverage_ratio
                  << " | health=" << coverage::ToString(coverage_summary.health)
                  << " | reason=" << coverage_summary.reason
                  << " | scan_window_duration_ms=" << scan_window.duration_ms
                  << " | scan_window_source="
                  << lidar::ToString(scan_window.source)
                  << " | scan_window_confidence="
                  << lidar::ToString(scan_window.confidence)
                  << " | scan_window_reason=" << scan_window.reason
                  << " | imu_buffer_size=" << imu_buffer_size);
}

}  // namespace

TemporalMonitorNode::TemporalMonitorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("temporal_monitor_node", options) {
  const std::string imu_topic =
      this->declare_parameter<std::string>("imu_topic", "/imu/data");
  const std::string lidar_topic =
      this->declare_parameter<std::string>("lidar_topic", "/points");

  const std::string map_update_allowed_topic =
      this->declare_parameter<std::string>(
          "map_update_allowed_topic", "/causal_slam/map_update_allowed");
  const std::string temporal_health_topic =
      this->declare_parameter<std::string>(
          "temporal_health_topic", "/causal_slam/temporal_health");
  const std::string map_update_reason_topic =
      this->declare_parameter<std::string>(
          "map_update_reason_topic", "/causal_slam/map_update_reason");
  const std::string fault_reasons_topic =
      this->declare_parameter<std::string>(
          "fault_reasons_topic", "/causal_slam/fault_reasons");

  const double summary_period_ms =
      this->declare_parameter<double>("summary_period_ms", 2000.0);
  const double safe_summary_period_ms = std::max(summary_period_ms, 100.0);

  const double imu_gap_threshold_ms =
      this->declare_parameter<double>("imu_gap_threshold_ms", 100.0);
  const double safe_imu_gap_threshold_ms =
      std::max(imu_gap_threshold_ms, 1.0);

  const double lidar_gap_threshold_ms =
      this->declare_parameter<double>("lidar_gap_threshold_ms", 500.0);
  const double safe_lidar_gap_threshold_ms =
      std::max(lidar_gap_threshold_ms, 1.0);

  const double lidar_scan_duration_ms =
      this->declare_parameter<double>("lidar_scan_duration_ms", 100.0);
  const double safe_lidar_scan_duration_ms =
      std::max(lidar_scan_duration_ms, 1.0);

  const double lidar_min_measured_scan_duration_ms =
      this->declare_parameter<double>(
          "lidar_min_measured_scan_duration_ms", 1.0);
  const double safe_lidar_min_measured_scan_duration_ms =
      std::max(lidar_min_measured_scan_duration_ms, 0.1);

  const double lidar_max_measured_scan_duration_ms =
      this->declare_parameter<double>(
          "lidar_max_measured_scan_duration_ms", 500.0);
  const double safe_lidar_max_measured_scan_duration_ms =
      std::max(lidar_max_measured_scan_duration_ms,
               safe_lidar_min_measured_scan_duration_ms);

  const bool lidar_prefer_measured_header_period =
      this->declare_parameter<bool>("lidar_prefer_measured_header_period", true);

  const std::string lidar_stamp_policy =
      this->declare_parameter<std::string>("lidar_stamp_policy", "scan_end");
  const auto parsed_lidar_stamp_policy =
      ParseLidarStampPolicy(lidar_stamp_policy);

  const double imu_buffer_retention_ms =
      this->declare_parameter<double>("imu_buffer_retention_ms", 5000.0);
  const double safe_imu_buffer_retention_ms =
      std::max(imu_buffer_retention_ms, 100.0);

  const double expected_imu_period_ms =
      this->declare_parameter<double>("expected_imu_period_ms", 5.0);
  const double safe_expected_imu_period_ms =
      std::max(expected_imu_period_ms, 0.1);

  const double max_missing_prefix_ms =
      this->declare_parameter<double>(
          "max_missing_prefix_ms", 2.0 * safe_expected_imu_period_ms);
  const double max_missing_suffix_ms =
      this->declare_parameter<double>(
          "max_missing_suffix_ms", 2.0 * safe_expected_imu_period_ms);
  const double max_internal_gap_ms =
      this->declare_parameter<double>(
          "max_internal_gap_ms", 5.0 * safe_expected_imu_period_ms);

  auto pipeline_config = pipeline::TemporalMonitorPipelineConfig{};
  pipeline_config.imu_gap_threshold_ms = safe_imu_gap_threshold_ms;
  pipeline_config.lidar_gap_threshold_ms = safe_lidar_gap_threshold_ms;
  pipeline_config.imu_buffer_retention_ns =
      MillisecondsToNanoseconds(safe_imu_buffer_retention_ms);
  pipeline_config.lidar_scan_window =
      lidar::LidarScanWindowEstimatorConfig{
          .fallback_scan_duration_ms = safe_lidar_scan_duration_ms,
          .min_measured_scan_duration_ms =
              safe_lidar_min_measured_scan_duration_ms,
          .max_measured_scan_duration_ms =
              safe_lidar_max_measured_scan_duration_ms,
          .stamp_policy = parsed_lidar_stamp_policy,
          .prefer_measured_header_period =
              lidar_prefer_measured_header_period,
      };
  pipeline_config.imu_coverage = coverage::ImuCoverageConfig{
      .max_missing_prefix_ms = std::max(max_missing_prefix_ms, 0.0),
      .max_missing_suffix_ms = std::max(max_missing_suffix_ms, 0.0),
      .max_internal_gap_ms =
          std::max(max_internal_gap_ms, safe_expected_imu_period_ms),
  };

  temporal_pipeline_.emplace(pipeline_config);

  imu_subscription_ =
      this->create_subscription<ImuMsg>(
          imu_topic, rclcpp::SensorDataQoS{},
          [this](ImuMsg::ConstSharedPtr msg) { OnImuReceived(msg); });

  lidar_subscription_ =
      this->create_subscription<PointCloud2Msg>(
          lidar_topic, rclcpp::SensorDataQoS{},
          [this](PointCloud2Msg::ConstSharedPtr msg) { OnLidarReceived(msg); });

  auto diagnostic_qos = rclcpp::QoS{1};
  diagnostic_qos.reliable();
  diagnostic_qos.transient_local();

  map_update_allowed_publisher_ =
      this->create_publisher<BoolMsg>(
          map_update_allowed_topic, diagnostic_qos);
  temporal_health_publisher_ =
      this->create_publisher<StringMsg>(
          temporal_health_topic, diagnostic_qos);
  map_update_reason_publisher_ =
      this->create_publisher<StringMsg>(
          map_update_reason_topic, diagnostic_qos);
  fault_reasons_publisher_ =
      this->create_publisher<StringMsg>(
          fault_reasons_topic, diagnostic_qos);

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double, std::milli>(safe_summary_period_ms)),
      [this]() { OnTimer(); });

  RCLCPP_INFO(this->get_logger(),
              "TemporalMonitorNode started"
              " | imu_topic=%s"
              " | lidar_topic=%s"
              " | map_update_allowed_topic=%s"
              " | temporal_health_topic=%s"
              " | map_update_reason_topic=%s"
              " | fault_reasons_topic=%s"
              " | summary_period_ms=%.3f"
              " | imu_gap_threshold_ms=%.3f"
              " | lidar_gap_threshold_ms=%.3f"
              " | lidar_scan_duration_ms=%.3f"
              " | lidar_min_measured_scan_duration_ms=%.3f"
              " | lidar_max_measured_scan_duration_ms=%.3f"
              " | lidar_prefer_measured_header_period=%s"
              " | lidar_stamp_policy=%s"
              " | expected_imu_period_ms=%.3f"
              " | imu_buffer_retention_ms=%.3f"
              " | max_missing_prefix_ms=%.3f"
              " | max_missing_suffix_ms=%.3f"
              " | max_internal_gap_ms=%.3f",
              imu_topic.c_str(), lidar_topic.c_str(),
              map_update_allowed_topic.c_str(), temporal_health_topic.c_str(),
              map_update_reason_topic.c_str(), fault_reasons_topic.c_str(),
              safe_summary_period_ms,
              safe_imu_gap_threshold_ms, safe_lidar_gap_threshold_ms,
              safe_lidar_scan_duration_ms,
              safe_lidar_min_measured_scan_duration_ms,
              safe_lidar_max_measured_scan_duration_ms,
              lidar_prefer_measured_header_period ? "true" : "false",
              lidar::ToString(parsed_lidar_stamp_policy),
              safe_expected_imu_period_ms, safe_imu_buffer_retention_ms,
              std::max(max_missing_prefix_ms, 0.0),
              std::max(max_missing_suffix_ms, 0.0),
              std::max(max_internal_gap_ms, safe_expected_imu_period_ms));
}

void TemporalMonitorNode::OnImuReceived(ImuMsg::ConstSharedPtr msg) {
  temporal_pipeline_->ObserveImu(pipeline::ImuPipelineInput{
      .header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds(),
      .receive_time_ns = this->now().nanoseconds(),
  });
}

void TemporalMonitorNode::OnLidarReceived(PointCloud2Msg::ConstSharedPtr msg) {
  temporal_pipeline_->ObserveLidar(pipeline::LidarPipelineInput{
      .header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds(),
      .receive_time_ns = this->now().nanoseconds(),
      .fields = ros_adapters::ToPointCloud2FieldInfos(*msg),
      .cloud_view = ros_adapters::ToPointCloud2CloudView(*msg),
  });
}

void TemporalMonitorNode::PublishDiagnosticTopics(
    const diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  BoolMsg map_update_allowed_msg;
  map_update_allowed_msg.data =
      snapshot.map_update_decision.map_update_allowed;
  map_update_allowed_publisher_->publish(map_update_allowed_msg);

  StringMsg temporal_health_msg;
  temporal_health_msg.data = telemetry::ToString(snapshot.overall_status);
  temporal_health_publisher_->publish(temporal_health_msg);

  StringMsg map_update_reason_msg;
  map_update_reason_msg.data =
      causal_slam::policy::ToString(snapshot.map_update_decision.reason);
  map_update_reason_publisher_->publish(map_update_reason_msg);

  StringMsg fault_reasons_msg;
  fault_reasons_msg.data = diagnostics::JoinFaultReasons(snapshot.issues);
  fault_reasons_publisher_->publish(fault_reasons_msg);
}

void TemporalMonitorNode::OnTimer() {
  const std::int64_t now_ns = this->now().nanoseconds();
  const auto snapshot = temporal_pipeline_->BuildSnapshot(now_ns);

  for (const auto& stream : snapshot.diagnostics.observation.streams) {
    LogTimingSummary(this->get_logger(), stream);
  }

  LogImuCoverageSummary(
      this->get_logger(),
      snapshot.diagnostics.observation.imu_coverage,
      snapshot.diagnostics.observation.lidar_scan_window,
      snapshot.diagnostics.observation.imu_buffer_size);

  PublishDiagnosticTopics(snapshot.diagnostics);

  const render::ConsoleTemporalSummaryRenderer renderer;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "\n" << renderer.Render(snapshot.diagnostics) << '\n'
                           << renderer.RenderStatistics(snapshot.statistics));
}

}  // namespace causal_slam::nodes
