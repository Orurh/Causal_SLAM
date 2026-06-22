#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "diagnostics/temporal_diagnostics.h"
#include "pipeline/temporal_monitor_pipeline.h"

namespace causal_slam::nodes {

class TemporalMonitorNode final : public rclcpp::Node {
 public:
  explicit TemporalMonitorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  using ImuMsg = sensor_msgs::msg::Imu;
  using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
  using BoolMsg = std_msgs::msg::Bool;
  using StringMsg = std_msgs::msg::String;

  void OnTimer();
  void OnImuReceived(ImuMsg::ConstSharedPtr msg);
  void OnLidarReceived(PointCloud2Msg::ConstSharedPtr msg);
  void PublishDiagnosticTopics(
      const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscription_;
  rclcpp::Subscription<PointCloud2Msg>::SharedPtr lidar_subscription_;

  rclcpp::Publisher<BoolMsg>::SharedPtr map_update_allowed_publisher_;
  rclcpp::Publisher<StringMsg>::SharedPtr temporal_health_publisher_;
  rclcpp::Publisher<StringMsg>::SharedPtr map_update_reason_publisher_;
  rclcpp::Publisher<StringMsg>::SharedPtr fault_reasons_publisher_;

  std::optional<causal_slam::pipeline::TemporalMonitorPipeline>
      temporal_pipeline_;
};

}  // namespace causal_slam::nodes
