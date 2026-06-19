#include "temporal_monitor_node.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace causal_slam::nodes {
namespace {

void LogTimingSummary(const rclcpp::Logger& logger, const char* stream_name, const causal_slam::telemetry::TimingSummary& summary) {
  RCLCPP_INFO_STREAM(
      logger, stream_name << " timing"
                          << " | total_count=" << summary.total_count << " | window_count=" << summary.window_count
                          << " | last_delay_ms=" << summary.last_delay_ms << " | window_avg_delay_ms=" << summary.window_average_delay_ms
                          << " | window_max_delay_ms=" << summary.window_max_delay_ms << " | last_period_ms=" << summary.last_period_ms
                          << " | last_jitter_ms=" << summary.last_jitter_ms << " | window_max_jitter_ms=" << summary.window_max_jitter_ms
                          << " | total_reordered_count=" << summary.total_reordered_count << " | window_reordered_count="
                          << summary.window_reordered_count << " | total_gap_count=" << summary.total_gap_count
                          << " | window_gap_count=" << summary.window_gap_count << " | max_gap_ms=" << summary.max_gap_ms
                          << " | health=" << causal_slam::telemetry::ToString(summary.health) << " | reason=" << summary.reason);
}

causal_slam::lidar::LidarStampPolicy ParseLidarStampPolicy(std::string_view value) {
  if (value == "scan_start") {
    return causal_slam::lidar::LidarStampPolicy::kScanStart;
  }

  if (value == "scan_middle") {
    return causal_slam::lidar::LidarStampPolicy::kScanMiddle;
  }

  if (value == "scan_end") {
    return causal_slam::lidar::LidarStampPolicy::kScanEnd;
  }

  return causal_slam::lidar::LidarStampPolicy::kScanEnd;
}

std::int64_t MillisecondsToNanoseconds(double milliseconds) {
  constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

  const double safe_milliseconds = std::max(milliseconds, 0.0);
  return static_cast<std::int64_t>(safe_milliseconds * kNanosecondsPerMillisecond);
}

void LogImuCoverageSummary(const rclcpp::Logger& logger, bool has_summary, const causal_slam::coverage::ImuCoverageSummary& summary,
                           const causal_slam::lidar::LidarScanWindowEstimate& scan_window_estimate, std::size_t imu_buffer_size) {
  if (!has_summary) {
    RCLCPP_INFO_STREAM(logger, "IMU coverage" << " | status=not_available"
                                              << " | reason=no_lidar_scan_received_yet"
                                              << " | imu_buffer_size=" << imu_buffer_size);
    return;
  }

  RCLCPP_INFO_STREAM(
      logger, "IMU coverage" << " | imu_count_in_window=" << summary.imu_count_in_window
                             << " | missing_prefix_ms=" << summary.missing_prefix_ms << " | missing_suffix_ms=" << summary.missing_suffix_ms
                             << " | max_gap_inside_ms=" << summary.max_gap_inside_ms << " | coverage_ratio=" << summary.coverage_ratio
                             << " | health=" << causal_slam::coverage::ToString(summary.health) << " | reason=" << summary.reason
                             << " | scan_window_duration_ms=" << scan_window_estimate.duration_ms
                             << " | scan_window_source=" << causal_slam::lidar::ToString(scan_window_estimate.source)
                             << " | scan_window_confidence=" << causal_slam::lidar::ToString(scan_window_estimate.confidence)
                             << " | scan_window_reason=" << scan_window_estimate.reason << " | imu_buffer_size=" << imu_buffer_size);
}

void LogPointCloud2FieldInspection(const rclcpp::Logger& logger, const causal_slam::ros_adapters::PointCloud2FieldInspection& inspection) {
  std::ostringstream fields_stream;

  for (std::size_t i = 0; i < inspection.fields.size(); ++i) {
    const auto& field = inspection.fields[i];

    if (i > 0) {
      fields_stream << ", ";
    }

    fields_stream << field.name << ":" << causal_slam::ros_adapters::PointCloud2DatatypeToString(field.datatype)
                  << "@offset=" << field.offset << ":count=" << field.count;

    if (field.time_role != causal_slam::ros_adapters::PointCloud2TimeFieldRole::kNone) {
      fields_stream << ":time_role=" << causal_slam::ros_adapters::ToString(field.time_role);
    }
  }

  RCLCPP_INFO_STREAM(logger, "LiDAR PointCloud2 fields"
                                 << " | field_count=" << inspection.fields.size()
                                 << " | has_time_candidate=" << (inspection.has_time_candidate ? "true" : "false")
                                 << " | has_supported_time_field=" << (inspection.has_supported_time_field ? "true" : "false")
                                 << " | reason=" << inspection.reason << " | fields=[" << fields_stream.str() << "]");

  if (inspection.primary_time_field.has_value()) {
    const auto& field = *inspection.primary_time_field;
    RCLCPP_INFO_STREAM(logger, "LiDAR PointCloud2 primary time field"
                                   << " | name=" << field.name << " | datatype="
                                   << causal_slam::ros_adapters::PointCloud2DatatypeToString(field.datatype) << " | offset=" << field.offset
                                   << " | count=" << field.count << " | role=" << causal_slam::ros_adapters::ToString(field.time_role));
  }
}

}  // namespace

TemporalMonitorNode::TemporalMonitorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("temporal_monitor_node", options) {
  const std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  const std::string lidar_topic = this->declare_parameter<std::string>("lidar_topic", "/points");

  const double summary_period_ms = this->declare_parameter<double>("summary_period_ms", 2000.0);
  const double safe_summary_period_ms = std::max(summary_period_ms, 100.0);

  const double imu_gap_threshold_ms = this->declare_parameter<double>("imu_gap_threshold_ms", 100.0);
  const double safe_imu_gap_threshold_ms = std::max(imu_gap_threshold_ms, 1.0);
  imu_timing_tracker_.SetGapThresholdMs(safe_imu_gap_threshold_ms);

  const double lidar_gap_threshold_ms = this->declare_parameter<double>("lidar_gap_threshold_ms", 500.0);
  const double safe_lidar_gap_threshold_ms = std::max(lidar_gap_threshold_ms, 1.0);
  lidar_timing_tracker_.SetGapThresholdMs(safe_lidar_gap_threshold_ms);

  const double lidar_scan_duration_ms = this->declare_parameter<double>("lidar_scan_duration_ms", 100.0);
  const double safe_lidar_scan_duration_ms = std::max(lidar_scan_duration_ms, 1.0);

  const double lidar_min_measured_scan_duration_ms = this->declare_parameter<double>("lidar_min_measured_scan_duration_ms", 1.0);
  const double safe_lidar_min_measured_scan_duration_ms = std::max(lidar_min_measured_scan_duration_ms, 0.1);

  const double lidar_max_measured_scan_duration_ms = this->declare_parameter<double>("lidar_max_measured_scan_duration_ms", 500.0);
  const double safe_lidar_max_measured_scan_duration_ms =
      std::max(lidar_max_measured_scan_duration_ms, safe_lidar_min_measured_scan_duration_ms);

  const bool lidar_prefer_measured_header_period = this->declare_parameter<bool>("lidar_prefer_measured_header_period", true);

  const std::string lidar_stamp_policy = this->declare_parameter<std::string>("lidar_stamp_policy", "scan_end");
  const auto parsed_lidar_stamp_policy = ParseLidarStampPolicy(lidar_stamp_policy);

  lidar_scan_window_estimator_.SetConfig(causal_slam::lidar::LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = safe_lidar_scan_duration_ms,
      .min_measured_scan_duration_ms = safe_lidar_min_measured_scan_duration_ms,
      .max_measured_scan_duration_ms = safe_lidar_max_measured_scan_duration_ms,
      .stamp_policy = parsed_lidar_stamp_policy,
      .prefer_measured_header_period = lidar_prefer_measured_header_period,
  });

  const double imu_buffer_retention_ms = this->declare_parameter<double>("imu_buffer_retention_ms", 5000.0);
  const double safe_imu_buffer_retention_ms = std::max(imu_buffer_retention_ms, 100.0);
  imu_sample_buffer_ = causal_slam::coverage::ImuSampleBuffer{MillisecondsToNanoseconds(safe_imu_buffer_retention_ms)};

  const double expected_imu_period_ms = this->declare_parameter<double>("expected_imu_period_ms", 5.0);
  const double safe_expected_imu_period_ms = std::max(expected_imu_period_ms, 0.1);

  const double max_missing_prefix_ms = this->declare_parameter<double>("max_missing_prefix_ms", 2.0 * safe_expected_imu_period_ms);
  const double max_missing_suffix_ms = this->declare_parameter<double>("max_missing_suffix_ms", 2.0 * safe_expected_imu_period_ms);
  const double max_internal_gap_ms = this->declare_parameter<double>("max_internal_gap_ms", 5.0 * safe_expected_imu_period_ms);

  imu_coverage_analyzer_.SetConfig(causal_slam::coverage::ImuCoverageConfig{
      .max_missing_prefix_ms = std::max(max_missing_prefix_ms, 0.0),
      .max_missing_suffix_ms = std::max(max_missing_suffix_ms, 0.0),
      .max_internal_gap_ms = std::max(max_internal_gap_ms, safe_expected_imu_period_ms),
  });

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS{}, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImuReceived(msg); });

  lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS{}, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { OnLidarReceived(msg); });

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_summary_period_ms)),
      [this]() { OnTimer(); });

  RCLCPP_INFO(this->get_logger(),
              "TemporalMonitorNode started"
              " | imu_topic=%s"
              " | lidar_topic=%s"
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
              imu_topic.c_str(), lidar_topic.c_str(), safe_summary_period_ms, safe_imu_gap_threshold_ms, safe_lidar_gap_threshold_ms,
              safe_lidar_scan_duration_ms, safe_lidar_min_measured_scan_duration_ms, safe_lidar_max_measured_scan_duration_ms,
              lidar_prefer_measured_header_period ? "true" : "false", causal_slam::lidar::ToString(parsed_lidar_stamp_policy),
              safe_expected_imu_period_ms, safe_imu_buffer_retention_ms, std::max(max_missing_prefix_ms, 0.0),
              std::max(max_missing_suffix_ms, 0.0), std::max(max_internal_gap_ms, safe_expected_imu_period_ms));
}

void TemporalMonitorNode::OnImuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

  imu_timing_tracker_.Observe(causal_slam::telemetry::TimingSample{
      .header_stamp_ns = stamp_ns,
      .receive_time_ns = this->now().nanoseconds(),
  });

  imu_sample_buffer_.Add(causal_slam::coverage::ImuSample{
      .stamp_ns = stamp_ns,
  });
}

void TemporalMonitorNode::OnLidarReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (!has_logged_lidar_point_cloud2_fields_) {
    const auto inspection = point_cloud2_field_inspector_.Inspect(*msg);
    LogPointCloud2FieldInspection(this->get_logger(), inspection);
    has_logged_lidar_point_cloud2_fields_ = true;
  }
  lidar_timing_tracker_.Observe(causal_slam::telemetry::TimingSample{
      .header_stamp_ns = stamp_ns,
      .receive_time_ns = this->now().nanoseconds(),
  });

  latest_lidar_scan_window_estimate_ = lidar_scan_window_estimator_.Estimate(stamp_ns);

  latest_imu_coverage_summary_ = imu_coverage_analyzer_.Analyze(latest_lidar_scan_window_estimate_.window, imu_sample_buffer_.Samples());

  has_lidar_coverage_summary_ = true;
}

void TemporalMonitorNode::OnTimer() {
  const auto imu_summary = imu_timing_tracker_.ConsumeWindowSummary();
  const auto lidar_summary = lidar_timing_tracker_.ConsumeWindowSummary();

  LogTimingSummary(this->get_logger(), "IMU", imu_summary);
  LogTimingSummary(this->get_logger(), "LiDAR", lidar_summary);
  LogImuCoverageSummary(this->get_logger(), has_lidar_coverage_summary_, latest_imu_coverage_summary_, latest_lidar_scan_window_estimate_,
                        imu_sample_buffer_.Size());
}

}  // namespace causal_slam::nodes