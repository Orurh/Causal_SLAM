#include "temporal_monitor_node.h"

#include <chrono>
#include <utility>

namespace causal_slam::nodes {

using namespace std::chrono_literals;

TemporalMonitorNode::TemporalMonitorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("temporal_monitor_node", options) {
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS{}, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImuReceived(std::move(msg)); });

  timer_ = this->create_wall_timer(2s, [this]() { OnTimer(); });

  RCLCPP_INFO(this->get_logger(), "TemporalMonitorNode started");
}

void TemporalMonitorNode::OnImuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  imu_timing_tracker_.Observe(causal_slam::telemetry::ImuTimingSample{
      .header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds(),
      .receive_time_ns = this->now().nanoseconds(),
  });
}

void TemporalMonitorNode::OnTimer() {
  const auto summary = imu_timing_tracker_.ConsumeWindowSummary();

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Temporal summary"
                         << " | total_imu_count=" << summary.total_count
                         << " | window_imu_count=" << summary.window_count
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
                         << " | imu_health=" << causal_slam::telemetry::ToString(summary.health)
                         << " | reason=" << summary.reason);
}

}  // namespace causal_slam::nodes