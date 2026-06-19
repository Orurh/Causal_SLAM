#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "coverage/imu_coverage_analyzer.h"
#include "coverage/imu_sample_buffer.h"
#include "diagnostics/temporal_diagnostics.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "ros_adapters/point_cloud2_field_inspector.h"
#include "ros_adapters/point_cloud2_time_field_extractor.h"
#include "telemetry/stream_timing_tracker.h"

namespace causal_slam::nodes {

class TemporalMonitorNode final : public rclcpp::Node {
 public:
  explicit TemporalMonitorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  using ImuMsg = sensor_msgs::msg::Imu;
  using PointCloud2Msg = sensor_msgs::msg::PointCloud2;

  void OnTimer();
  void OnImuReceived(ImuMsg::ConstSharedPtr msg);
  void OnLidarReceived(PointCloud2Msg::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscription_;
  rclcpp::Subscription<PointCloud2Msg>::SharedPtr lidar_subscription_;

  causal_slam::telemetry::StreamTimingTracker imu_timing_tracker_;
  causal_slam::telemetry::StreamTimingTracker lidar_timing_tracker_;

  causal_slam::lidar::LidarScanWindowEstimator lidar_scan_window_estimator_;
  causal_slam::lidar::LidarScanWindowEstimate latest_lidar_scan_window_estimate_;

  causal_slam::coverage::ImuSampleBuffer imu_sample_buffer_{5'000'000'000LL};
  causal_slam::coverage::ImuCoverageAnalyzer imu_coverage_analyzer_;
  causal_slam::coverage::ImuCoverageSummary latest_imu_coverage_summary_;
  bool has_lidar_coverage_summary_{false};

  causal_slam::ros_adapters::PointCloud2FieldInspector point_cloud2_field_inspector_;
  causal_slam::ros_adapters::PointCloud2TimeFieldExtractor
      point_cloud2_time_field_extractor_;

  std::optional<causal_slam::ros_adapters::PointCloud2FieldInfo>
      lidar_point_time_field_;

  causal_slam::diagnostics::PointTimeDiagnostics
      latest_lidar_point_time_diagnostics_;

  bool has_logged_lidar_point_cloud2_fields_{false};
};

}  // namespace causal_slam::nodes