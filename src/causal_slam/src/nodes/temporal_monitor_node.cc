#include "temporal_monitor_node.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

namespace causal_slam::nodes {

namespace {

void LogTimingSummary(rclcpp::Logger logger, const char* stream_name, const causal_slam::telemetry::TimingSummary& summary) {
  RCLCPP_INFO_STREAM(
      logger,
      stream_name << " timing"
                  << " | total_count=" << summary.total_count << " | window_count=" << summary.window_count
                  << " | last_delay_ms=" << summary.last_delay_ms
                  << " | window_avg_delay_ms=" << summary.window_average_delay_ms
                  << " | window_max_delay_ms=" << summary.window_max_delay_ms
                  << " | last_period_ms=" << summary.last_period_ms
                  << " | last_jitter_ms=" << summary.last_jitter_ms
                  << " | window_max_jitter_ms=" << summary.window_max_jitter_ms
                  << " | total_reordered_count=" << summary.total_reordered_count
                  << " | window_reordered_count=" << summary.window_reordered_count
                  << " | total_gap_count=" << summary.total_gap_count
                  << " | window_gap_count=" << summary.window_gap_count
                  << " | max_gap_ms=" << summary.max_gap_ms
                  << " | health=" << causal_slam::telemetry::ToString(summary.health)
                  << " | reason=" << summary.reason);
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

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS{}, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImuReceived(std::move(msg)); });

  lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS{},
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { OnLidarReceived(std::move(msg)); });

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_summary_period_ms)),
      [this]() { OnTimer(); });

  RCLCPP_INFO(this->get_logger(),
              "TemporalMonitorNode started | imu_topic=%s | lidar_topic=%s | summary_period_ms=%.3f | imu_gap_threshold_ms=%.3f | "
              "lidar_gap_threshold_ms=%.3f",
              imu_topic.c_str(), lidar_topic.c_str(), safe_summary_period_ms, safe_imu_gap_threshold_ms, safe_lidar_gap_threshold_ms);
}

void TemporalMonitorNode::OnImuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  imu_timing_tracker_.Observe(causal_slam::telemetry::TimingSample{
      .header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds(),
      .receive_time_ns = this->now().nanoseconds(),
  });
}

void TemporalMonitorNode::OnLidarReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  lidar_timing_tracker_.Observe(causal_slam::telemetry::TimingSample{
      .header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds(),
      .receive_time_ns = this->now().nanoseconds(),
  });
}

void TemporalMonitorNode::OnTimer() {
  const auto imu_summary = imu_timing_tracker_.ConsumeWindowSummary();
  const auto lidar_summary = lidar_timing_tracker_.ConsumeWindowSummary();

  LogTimingSummary(this->get_logger(), "IMU", imu_summary);
  LogTimingSummary(this->get_logger(), "LiDAR", lidar_summary);
}

}  // namespace causal_slam::nodes