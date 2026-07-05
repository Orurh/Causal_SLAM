#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "application/temporal_monitor/temporal_monitor_pipeline.h"
#include "domain/diagnostics/temporal_diagnostics.h"
#include "domain/policy/lidar_cloud_gate.h"
#include "temporal_monitor_node_parameters.h"

namespace causal_slam::nodes {

class TemporalMonitorNode final : public rclcpp::Node {
 public:
  explicit TemporalMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  using ImuMsg = sensor_msgs::msg::Imu;
  using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
  using BoolMsg = std_msgs::msg::Bool;
  using StringMsg = std_msgs::msg::String;

  void OnTimer();
  void OnImuReceived(ImuMsg::ConstSharedPtr msg);
  void OnLidarReceived(PointCloud2Msg::ConstSharedPtr msg);
  void ProcessLidarCloud(PointCloud2Msg::ConstSharedPtr msg, std::int64_t receive_time_ns,
                         std::optional<causal_slam::lidar::LidarScanWindowEstimate> precomputed_scan_window_estimate);
  void ProcessReadyPendingLidarClouds();
  void DropOldestPendingLidarCloudsIfNeeded();
  void ObserveConfiguredTransformsForLidar(const PointCloud2Msg& msg, std::int64_t header_stamp_ns, std::int64_t receive_time_ns);
  void PublishDiagnosticTopics(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                               const causal_slam::policy::MapUpdateDecision& map_update_decision, bool publish_decision_json);
  [[nodiscard]] causal_slam::statistics::CloudForwardingDecision MaybePublishCheckedLidar(
      const PointCloud2Msg& msg, const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot);
  [[nodiscard]] std::int64_t ComputeLidarHoldbackReadyStampNs(
      const PointCloud2Msg& msg, std::optional<causal_slam::lidar::LidarScanWindowEstimate>* precomputed_scan_window_estimate);

  struct PendingLidarCloud {
    PointCloud2Msg::ConstSharedPtr msg;
    std::int64_t header_stamp_ns{0};
    std::int64_t receive_time_ns{0};
    std::int64_t ready_imu_stamp_ns{0};
    std::optional<causal_slam::lidar::LidarScanWindowEstimate> precomputed_scan_window_estimate;
  };

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscription_;
  rclcpp::Subscription<PointCloud2Msg>::SharedPtr lidar_subscription_;

  bool tf_monitoring_enabled_{false};
  std::vector<TransformCheckConfig> transform_checks_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<BoolMsg>::SharedPtr map_update_allowed_publisher_;
  rclcpp::Publisher<StringMsg>::SharedPtr temporal_health_publisher_;
  rclcpp::Publisher<StringMsg>::SharedPtr map_update_reason_publisher_;
  rclcpp::Publisher<StringMsg>::SharedPtr fault_reasons_publisher_;
  rclcpp::Publisher<StringMsg>::SharedPtr map_update_decision_json_publisher_;
  rclcpp::Publisher<PointCloud2Msg>::SharedPtr checked_lidar_publisher_;

  std::string html_report_path_;
  causal_slam::policy::LidarCloudGateConfig lidar_gate_config_;
  RuntimeProfile runtime_profile_{RuntimeProfile::kDebugReport};

  std::uint64_t cloud_decision_sequence_id_{0};

  bool lidar_holdback_enabled_{false};
  std::int64_t lidar_holdback_scan_duration_ns_{100'000'000LL};
  std::int64_t lidar_holdback_tolerance_ns_{10'000'000LL};
  std::size_t lidar_holdback_max_pending_{32};
  std::optional<std::int64_t> latest_imu_header_stamp_ns_;
  std::deque<PendingLidarCloud> pending_lidar_clouds_;

  causal_slam::lidar::LidarScanWindowEstimator holdback_scan_window_estimator_;
  causal_slam::pointcloud::PointCloud2FieldInspector holdback_point_cloud2_field_inspector_;
  causal_slam::pointcloud::PointCloud2TimeFieldExtractor holdback_point_cloud2_time_field_extractor_;
  causal_slam::pointcloud::PointCloud2TimeFieldOverrideConfig holdback_point_time_config_;
  std::optional<causal_slam::pointcloud::PointCloud2FieldInfo> holdback_point_time_field_;

  std::optional<causal_slam::pipeline::TemporalMonitorPipeline> temporal_pipeline_;
};

}  // namespace causal_slam::nodes
