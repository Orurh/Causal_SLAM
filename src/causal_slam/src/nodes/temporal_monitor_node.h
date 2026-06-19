#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include "telemetry/imu_timing_tracker.h"

namespace causal_slam::nodes {

class TemporalMonitorNode final : public rclcpp::Node {
 public:
  explicit TemporalMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  void OnTimer();
  void OnImuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  causal_slam::telemetry::ImuTimingTracker imu_timing_tracker_;
};

}  // namespace causal_slam::nodes
