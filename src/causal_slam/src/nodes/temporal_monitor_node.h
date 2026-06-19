#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "telemetry/stream_timing_tracker.h"

namespace causal_slam::nodes {

class TemporalMonitorNode final : public rclcpp::Node {
 public:
  explicit TemporalMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  void OnTimer();
  void OnImuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void OnLidarReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;

  causal_slam::telemetry::StreamTimingTracker imu_timing_tracker_;
  causal_slam::telemetry::StreamTimingTracker lidar_timing_tracker_;
};

}  // namespace causal_slam::nodes