#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "causal_slam_ros1/point_cloud2_conversions.h"
#include "domain/sensors/imu/imu_coverage_analyzer.h"
#include "domain/diagnostics/temporal_fault_reason_formatter.h"
#include "domain/sensors/lidar/lidar_scan_window_estimator.h"
#include "application/temporal_monitor/temporal_monitor_pipeline.h"
#include "domain/policy/map_update_decision.h"
#include "presentation/render/console_temporal_summary_renderer.h"
#include "domain/telemetry/temporal_health.h"

namespace {

namespace coverage = causal_slam::coverage;
namespace lidar = causal_slam::lidar;
namespace pipeline = causal_slam::pipeline;
namespace render = causal_slam::render;
namespace telemetry = causal_slam::telemetry;

template <typename T>
T GetParam(ros::NodeHandle& node, const std::string& name, const T& fallback) {
  T value{};
  node.param<T>(name, value, fallback);
  return value;
}

std::int64_t NowNanoseconds() {
  return static_cast<std::int64_t>(ros::Time::now().toNSec());
}

std::int64_t MillisecondsToNanoseconds(double milliseconds) {
  constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

  const double safe_milliseconds = std::max(milliseconds, 0.0);
  return static_cast<std::int64_t>(
      safe_milliseconds * kNanosecondsPerMillisecond);
}

lidar::LidarStampPolicy ParseLidarStampPolicy(const std::string& value) {
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

pipeline::TemporalMonitorPipelineConfig LoadPipelineConfig(
    ros::NodeHandle& private_node) {
  const double imu_gap_threshold_ms =
      GetParam(private_node, "imu_gap_threshold_ms", 100.0);
  const double lidar_gap_threshold_ms =
      GetParam(private_node, "lidar_gap_threshold_ms", 500.0);

  const double lidar_scan_duration_ms =
      GetParam(private_node, "lidar_scan_duration_ms", 100.0);
  const double lidar_min_measured_scan_duration_ms =
      GetParam(private_node, "lidar_min_measured_scan_duration_ms", 1.0);
  const double lidar_max_measured_scan_duration_ms =
      GetParam(private_node, "lidar_max_measured_scan_duration_ms", 500.0);
  const bool lidar_prefer_measured_header_period =
      GetParam(private_node, "lidar_prefer_measured_header_period", true);
  const std::string lidar_stamp_policy =
      GetParam<std::string>(private_node, "lidar_stamp_policy", "scan_end");

  const double imu_buffer_retention_ms =
      GetParam(private_node, "imu_buffer_retention_ms", 5000.0);
  const double expected_imu_period_ms =
      GetParam(private_node, "expected_imu_period_ms", 5.0);
  const double safe_expected_imu_period_ms =
      std::max(expected_imu_period_ms, 0.1);

  const double max_missing_prefix_ms =
      GetParam(private_node, "max_missing_prefix_ms",
               2.0 * safe_expected_imu_period_ms);
  const double max_missing_suffix_ms =
      GetParam(private_node, "max_missing_suffix_ms",
               2.0 * safe_expected_imu_period_ms);
  const double max_internal_gap_ms =
      GetParam(private_node, "max_internal_gap_ms",
               5.0 * safe_expected_imu_period_ms);

  pipeline::TemporalMonitorPipelineConfig config;
  config.imu_gap_threshold_ms = std::max(imu_gap_threshold_ms, 1.0);
  config.lidar_gap_threshold_ms = std::max(lidar_gap_threshold_ms, 1.0);
  config.imu_buffer_retention_ns =
      MillisecondsToNanoseconds(std::max(imu_buffer_retention_ms, 100.0));

  const double safe_lidar_min_measured_scan_duration_ms =
      std::max(lidar_min_measured_scan_duration_ms, 0.1);
  const double safe_lidar_max_measured_scan_duration_ms =
      std::max(lidar_max_measured_scan_duration_ms,
               safe_lidar_min_measured_scan_duration_ms);

  config.lidar_scan_window = lidar::LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = std::max(lidar_scan_duration_ms, 1.0),
      .min_measured_scan_duration_ms =
          safe_lidar_min_measured_scan_duration_ms,
      .max_measured_scan_duration_ms =
          safe_lidar_max_measured_scan_duration_ms,
      .stamp_policy = ParseLidarStampPolicy(lidar_stamp_policy),
      .prefer_measured_header_period = lidar_prefer_measured_header_period,
  };

  config.imu_coverage = coverage::ImuCoverageConfig{
      .max_missing_prefix_ms = std::max(max_missing_prefix_ms, 0.0),
      .max_missing_suffix_ms = std::max(max_missing_suffix_ms, 0.0),
      .max_internal_gap_ms =
          std::max(max_internal_gap_ms, safe_expected_imu_period_ms),
  };

  return config;
}

class TemporalMonitorRos1Node final {
 public:
  TemporalMonitorRos1Node(ros::NodeHandle node, ros::NodeHandle private_node)
      : node_(std::move(node)),
        private_node_(std::move(private_node)),
        temporal_pipeline_(LoadPipelineConfig(private_node_)) {
    const std::string imu_topic =
        GetParam<std::string>(private_node_, "imu_topic", "/imu/data");
    const std::string lidar_topic =
        GetParam<std::string>(private_node_, "lidar_topic", "/points");

    const std::string map_update_allowed_topic =
        GetParam<std::string>(
            private_node_, "map_update_allowed_topic",
            "/causal_slam/map_update_allowed");
    const std::string temporal_health_topic =
        GetParam<std::string>(
            private_node_, "temporal_health_topic",
            "/causal_slam/temporal_health");
    const std::string map_update_reason_topic =
        GetParam<std::string>(
            private_node_, "map_update_reason_topic",
            "/causal_slam/map_update_reason");
    const std::string fault_reasons_topic =
        GetParam<std::string>(
            private_node_, "fault_reasons_topic",
            "/causal_slam/fault_reasons");

    const double summary_period_ms =
        GetParam(private_node_, "summary_period_ms", 2000.0);
    const double safe_summary_period_ms = std::max(summary_period_ms, 100.0);

    imu_subscription_ = node_.subscribe(
        imu_topic, 200, &TemporalMonitorRos1Node::OnImuReceived, this,
        ros::TransportHints().tcpNoDelay());

    lidar_subscription_ = node_.subscribe(
        lidar_topic, 20, &TemporalMonitorRos1Node::OnLidarReceived, this,
        ros::TransportHints().tcpNoDelay());

    constexpr bool kLatchDiagnosticTopics = true;

    map_update_allowed_publisher_ =
        node_.advertise<std_msgs::Bool>(
            map_update_allowed_topic, 1, kLatchDiagnosticTopics);
    temporal_health_publisher_ =
        node_.advertise<std_msgs::String>(
            temporal_health_topic, 1, kLatchDiagnosticTopics);
    map_update_reason_publisher_ =
        node_.advertise<std_msgs::String>(
            map_update_reason_topic, 1, kLatchDiagnosticTopics);
    fault_reasons_publisher_ =
        node_.advertise<std_msgs::String>(
            fault_reasons_topic, 1, kLatchDiagnosticTopics);

    timer_ = node_.createTimer(
        ros::Duration(safe_summary_period_ms / 1000.0),
        &TemporalMonitorRos1Node::OnTimer, this);

    ROS_INFO_STREAM("causal_slam_ros1 temporal monitor started"
                    << " | imu_topic=" << imu_topic
                    << " | lidar_topic=" << lidar_topic
                    << " | map_update_allowed_topic="
                    << map_update_allowed_topic
                    << " | temporal_health_topic=" << temporal_health_topic
                    << " | map_update_reason_topic="
                    << map_update_reason_topic
                    << " | fault_reasons_topic=" << fault_reasons_topic
                    << " | summary_period_ms="
                    << safe_summary_period_ms);
  }

 private:
  void OnImuReceived(const sensor_msgs::ImuConstPtr& msg) {
    temporal_pipeline_.ObserveImu(pipeline::ImuPipelineInput{
        .header_stamp_ns =
            causal_slam_ros1::StampToNanoseconds(msg->header.stamp),
        .receive_time_ns = NowNanoseconds(),
    });
  }

  void OnLidarReceived(const sensor_msgs::PointCloud2ConstPtr& msg) {
    temporal_pipeline_.ObserveLidar(pipeline::LidarPipelineInput{
        .header_stamp_ns =
            causal_slam_ros1::StampToNanoseconds(msg->header.stamp),
        .receive_time_ns = NowNanoseconds(),
        .fields = causal_slam_ros1::ToPointCloud2FieldInfos(*msg),
        .cloud_view = causal_slam_ros1::ToPointCloud2CloudView(*msg),
    });
  }

  void OnTimer(const ros::TimerEvent&) {
    const auto snapshot = temporal_pipeline_.BuildSnapshot(NowNanoseconds());

     PublishDiagnosticTopics(snapshot.diagnostics, snapshot.map_update_decision);

    const render::ConsoleTemporalSummaryRenderer renderer;
    ROS_INFO_STREAM("\n"
                    << renderer.Render(snapshot.diagnostics,
                                       snapshot.map_update_decision)
                    << '\n'
                    << renderer.RenderStatistics(snapshot.statistics));

  }

void PublishDiagnosticTopics(
    const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
    const causal_slam::policy::MapUpdateDecision& map_update_decision) {
    std_msgs::Bool map_update_allowed_msg;
    map_update_allowed_msg.data =
        map_update_decision.map_update_allowed;
    map_update_allowed_publisher_.publish(map_update_allowed_msg);

    std_msgs::String temporal_health_msg;
    temporal_health_msg.data = telemetry::ToString(snapshot.overall_status);
    temporal_health_publisher_.publish(temporal_health_msg);

    std_msgs::String map_update_reason_msg;
    map_update_reason_msg.data =
        causal_slam::policy::ToString(map_update_decision.reason);
    map_update_reason_publisher_.publish(map_update_reason_msg);

    std_msgs::String fault_reasons_msg;
    fault_reasons_msg.data =
        causal_slam::diagnostics::JoinFaultReasons(snapshot.issues);
    fault_reasons_publisher_.publish(fault_reasons_msg);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;

  pipeline::TemporalMonitorPipeline temporal_pipeline_;

  ros::Subscriber imu_subscription_;
  ros::Subscriber lidar_subscription_;

  ros::Publisher map_update_allowed_publisher_;
  ros::Publisher temporal_health_publisher_;
  ros::Publisher map_update_reason_publisher_;
  ros::Publisher fault_reasons_publisher_;

  ros::Timer timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "causal_slam_ros1_temporal_monitor_node");

  ros::NodeHandle node;
  ros::NodeHandle private_node{"~"};

  TemporalMonitorRos1Node monitor{node, private_node};

  ros::spin();
  return 0;
}
