#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "coverage/imu_coverage_analyzer.h"
#include "coverage/imu_sample_buffer.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "ros_adapters/point_cloud2_field_inspector.h"
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

  causal_slam::lidar::LidarScanWindowEstimator lidar_scan_window_estimator_;
  causal_slam::lidar::LidarScanWindowEstimate latest_lidar_scan_window_estimate_;

  causal_slam::telemetry::StreamTimingTracker imu_timing_tracker_;
  causal_slam::telemetry::StreamTimingTracker lidar_timing_tracker_;

  causal_slam::coverage::ImuSampleBuffer imu_sample_buffer_{5'000'000'000LL};
  causal_slam::coverage::ImuCoverageAnalyzer imu_coverage_analyzer_;

  causal_slam::coverage::ImuCoverageSummary latest_imu_coverage_summary_;
  bool has_lidar_coverage_summary_{false};

  causal_slam::ros_adapters::PointCloud2FieldInspector point_cloud2_field_inspector_;
  bool has_logged_lidar_point_cloud2_fields_{false};
};

}  // namespace causal_slam::nodes