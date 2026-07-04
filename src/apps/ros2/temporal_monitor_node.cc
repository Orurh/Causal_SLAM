#include "temporal_monitor_node.h"
#include "adapters/ros2/support/point_cloud_qos.h"
#include "temporal_monitor_node_parameters.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "adapters/ros2/point_cloud2_conversions.h"
#include "adapters/ros2/transform_lookup_adapter.h"
#include "domain/diagnostics/temporal_fault_reason_formatter.h"
#include "domain/model/temporal_observation.h"
#include "domain/policy/lidar_cloud_gate.h"
#include "domain/policy/map_update_decision.h"
#include "domain/sensors/imu/imu_coverage_analyzer.h"
#include "domain/sensors/lidar/lidar_scan_window_estimator.h"
#include "platform/atomic_file_writer.h"
#include "presentation/render/console_temporal_summary_renderer.h"
#include "presentation/render/html_temporal_summary_renderer.h"
#include "presentation/render/map_update_decision_json_renderer.h"

namespace causal_slam::nodes {

namespace coverage = causal_slam::coverage;
namespace diagnostics = causal_slam::diagnostics;
namespace lidar = causal_slam::lidar;
namespace pipeline = causal_slam::pipeline;
namespace platform = causal_slam::platform;
namespace policy = causal_slam::policy;
namespace render = causal_slam::render;
namespace ros_adapters = causal_slam::ros_adapters;
namespace telemetry = causal_slam::telemetry;

namespace {

void LogTimingSummary(const rclcpp::Logger& logger, const telemetry::StreamTimingDiagnostic& stream) {
  const auto& summary = stream.timing;

  RCLCPP_INFO_STREAM(logger,
                     telemetry::ToString(stream.id)
                         << " timing"
                         << " | total_count=" << summary.total_count << " | window_count=" << summary.window_count
                         << " | last_delay_ms=" << summary.last_delay_ms << " | window_avg_delay_ms=" << summary.window_average_delay_ms
                         << " | window_max_delay_ms=" << summary.window_max_delay_ms << " | last_period_ms=" << summary.last_period_ms
                         << " | last_jitter_ms=" << summary.last_jitter_ms << " | window_max_jitter_ms=" << summary.window_max_jitter_ms
                         << " | total_reordered_count=" << summary.total_reordered_count << " | window_reordered_count="
                         << summary.window_reordered_count << " | total_gap_count=" << summary.total_gap_count
                         << " | window_gap_count=" << summary.window_gap_count << " | max_gap_ms=" << summary.max_gap_ms
                         << " | health=" << telemetry::ToString(summary.health) << " | reason=" << summary.reason);
}

std::string ResolveSourceFrame(const std::string& configured_source_frame, const std::string& cloud_frame_id) {
  if (configured_source_frame.empty() || configured_source_frame == "<cloud_frame>" || configured_source_frame == "cloud_frame") {
    return cloud_frame_id;
  }

  return configured_source_frame;
}

policy::LidarCloudGateInput BuildLidarCloudGateInput(const diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  policy::LidarCloudGateInput input{
      .health = snapshot.overall_status,
  };

  const auto imu_stream = std::find_if(snapshot.observation.streams.begin(), snapshot.observation.streams.end(),
                                       [](const auto& stream) { return stream.id == telemetry::TemporalStreamId::kImu; });

  if (imu_stream != snapshot.observation.streams.end()) {
    input.total_imu_samples = imu_stream->timing.total_count;
    input.window_imu_samples = imu_stream->timing.window_count;
  }

  return input;
}

void LogImuCoverageSummary(const rclcpp::Logger& logger, const std::optional<coverage::ImuCoverageSummary>& summary,
                           const std::optional<lidar::LidarScanWindowEstimate>& scan_window_estimate, std::size_t imu_buffer_size) {
  if (!summary.has_value() || !scan_window_estimate.has_value()) {
    RCLCPP_INFO_STREAM(logger, "IMU coverage" << " | status=not_available"
                                              << " | reason=no_lidar_scan_received_yet"
                                              << " | imu_buffer_size=" << imu_buffer_size);
    return;
  }

  const auto& coverage_summary = *summary;
  const auto& scan_window = *scan_window_estimate;

  RCLCPP_INFO_STREAM(logger, "IMU coverage" << " | imu_count_in_window=" << coverage_summary.imu_count_in_window
                                            << " | missing_prefix_ms=" << coverage_summary.missing_prefix_ms
                                            << " | missing_suffix_ms=" << coverage_summary.missing_suffix_ms << " | max_gap_inside_ms="
                                            << coverage_summary.max_gap_inside_ms << " | coverage_ratio=" << coverage_summary.coverage_ratio
                                            << " | health=" << coverage::ToString(coverage_summary.health) << " | reason="
                                            << coverage_summary.reason << " | scan_window_duration_ms=" << scan_window.duration_ms
                                            << " | scan_window_source=" << lidar::ToString(scan_window.source)
                                            << " | scan_window_confidence=" << lidar::ToString(scan_window.confidence)
                                            << " | scan_window_reason=" << scan_window.reason << " | imu_buffer_size=" << imu_buffer_size);
}

causal_slam::lidar::LidarScanWindowEstimate BuildHoldbackPointTimeScanWindowEstimate(
    const causal_slam::pointcloud::PointCloud2TimeFieldExtraction& extraction) {
  return causal_slam::lidar::LidarScanWindowEstimate{
      .window = extraction.scan_window,
      .duration_ms = static_cast<double>(extraction.scan_window.DurationNs()) / 1'000'000.0,
      .source = causal_slam::lidar::LidarScanWindowSource::kPointTimeField,
      .confidence = causal_slam::lidar::LidarScanWindowConfidence::kHigh,
      .reason = extraction.reason,
  };
}

}  // namespace

TemporalMonitorNode::TemporalMonitorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("temporal_monitor_node", options) {
  const auto params = LoadTemporalMonitorNodeParameters(*this);

  runtime_profile_ = params.runtime_profile;
  html_report_path_ = params.html_report_path;
  lidar_gate_config_ = policy::LidarCloudGateConfig{
      .mode = policy::ParseLidarGateMode(params.lidar_gate_mode),
      .min_total_imu_samples_before_forward = params.gate_min_total_imu_samples_before_forward,
      .min_window_imu_samples_before_forward = params.gate_min_window_imu_samples_before_forward,
  };
  lidar_holdback_enabled_ = params.lidar_holdback_enabled;
  lidar_holdback_scan_duration_ns_ = static_cast<std::int64_t>(std::llround(params.lidar_scan_duration_ms * 1'000'000.0));
  lidar_holdback_tolerance_ns_ = static_cast<std::int64_t>(std::llround(params.lidar_holdback_tolerance_ms * 1'000'000.0));
  lidar_holdback_max_pending_ = static_cast<std::size_t>(std::max(params.lidar_holdback_max_pending, 1));
  holdback_scan_window_estimator_.SetConfig(params.pipeline_config.lidar_scan_window);
  holdback_point_time_config_ = params.pipeline_config.point_time;
  tf_monitoring_enabled_ = params.tf_monitoring_enabled;
  transform_checks_ = params.transform_checks;

  temporal_pipeline_.emplace(params.pipeline_config);

  if (tf_monitoring_enabled_ && transform_checks_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "TF monitoring requested, but tf_target_frames/tf_source_frames are "
                "empty or have different sizes. TF monitoring is disabled.");
    tf_monitoring_enabled_ = false;
  }

  if (tf_monitoring_enabled_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  imu_subscription_ = this->create_subscription<ImuMsg>(params.imu_topic, rclcpp::SensorDataQoS{},
                                                        [this](ImuMsg::ConstSharedPtr msg) { OnImuReceived(msg); });

  lidar_subscription_ = this->create_subscription<PointCloud2Msg>(
      params.lidar_topic, causal_slam::ros_support::MakePointCloudQos(params.lidar_qos_reliability, params.lidar_qos_depth),
      [this](PointCloud2Msg::ConstSharedPtr msg) { OnLidarReceived(msg); });

  auto diagnostic_qos = rclcpp::QoS{1};
  diagnostic_qos.reliable();
  diagnostic_qos.transient_local();

  map_update_allowed_publisher_ = this->create_publisher<BoolMsg>(params.map_update_allowed_topic, diagnostic_qos);
  temporal_health_publisher_ = this->create_publisher<StringMsg>(params.temporal_health_topic, diagnostic_qos);
  map_update_reason_publisher_ = this->create_publisher<StringMsg>(params.map_update_reason_topic, diagnostic_qos);
  fault_reasons_publisher_ = this->create_publisher<StringMsg>(params.fault_reasons_topic, diagnostic_qos);
  map_update_decision_json_publisher_ = this->create_publisher<StringMsg>(params.map_update_decision_json_topic, diagnostic_qos);

  if (!params.checked_lidar_topic.empty()) {
    checked_lidar_publisher_ = this->create_publisher<PointCloud2Msg>(
        params.checked_lidar_topic,
        causal_slam::ros_support::MakePointCloudQos(params.checked_lidar_qos_reliability, params.checked_lidar_qos_depth));
  }

  RCLCPP_INFO(this->get_logger(),
              "TemporalMonitor LiDAR QoS"
              " | lidar_qos_reliability=%s"
              " | lidar_qos_depth=%d"
              " | checked_lidar_qos_reliability=%s"
              " | checked_lidar_qos_depth=%d",
              params.lidar_qos_reliability.c_str(), params.lidar_qos_depth, params.checked_lidar_qos_reliability.c_str(),
              params.checked_lidar_qos_depth);

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(params.summary_period_ms)),
      [this]() { OnTimer(); });

  RCLCPP_INFO(this->get_logger(),
              "TemporalMonitorNode started"
              " | imu_topic=%s"
              " | lidar_topic=%s"
              " | checked_lidar_topic=%s"
              " | lidar_gate_mode=%s"
              " | map_update_allowed_topic=%s"
              " | temporal_health_topic=%s"
              " | map_update_reason_topic=%s"
              " | fault_reasons_topic=%s"
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
              params.imu_topic.c_str(), params.lidar_topic.c_str(),
              params.checked_lidar_topic.empty() ? "<disabled>" : params.checked_lidar_topic.c_str(),
              policy::ToString(lidar_gate_config_.mode), params.map_update_allowed_topic.c_str(), params.temporal_health_topic.c_str(),
              params.map_update_reason_topic.c_str(), params.fault_reasons_topic.c_str(), params.summary_period_ms,
              params.imu_gap_threshold_ms, params.lidar_gap_threshold_ms, params.lidar_scan_duration_ms,
              params.lidar_min_measured_scan_duration_ms, params.lidar_max_measured_scan_duration_ms,
              params.lidar_prefer_measured_header_period ? "true" : "false", lidar::ToString(params.lidar_stamp_policy),
              params.expected_imu_period_ms, params.imu_buffer_retention_ms, params.max_missing_prefix_ms, params.max_missing_suffix_ms,
              params.max_internal_gap_ms);

  RCLCPP_INFO(this->get_logger(),
              "TF monitoring"
              " | enabled=%s"
              " | check_count=%zu"
              " | tf_max_transform_age_ms=%.3f"
              " | tf_max_future_tolerance_ms=%.3f",
              tf_monitoring_enabled_ ? "true" : "false", transform_checks_.size(), params.tf_max_transform_age_ms,
              params.tf_max_future_tolerance_ms);

  RCLCPP_INFO(this->get_logger(),
              "HTML report"
              " | enabled=%s"
              " | path=%s",
              html_report_path_.empty() ? "false" : "true", html_report_path_.c_str());
}

namespace {

std::vector<std::string> FaultReasonStrings(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  std::vector<std::string> reasons;
  reasons.reserve(snapshot.issues.size());

  for (const auto& issue : snapshot.issues) {
    const std::string reason = causal_slam::diagnostics::ToString(issue.reason);
    if (reason != "none") {
      reasons.push_back(reason);
    }
  }

  return reasons;
}

std::string PrimaryBlockReason(causal_slam::statistics::CloudForwardingDecision decision,
                               const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                               const causal_slam::policy::MapUpdateDecision& map_update_decision) {
  if (decision == causal_slam::statistics::CloudForwardingDecision::kForwarded) {
    return "forwarded";
  }

  if (decision == causal_slam::statistics::CloudForwardingDecision::kBlockedWarmup) {
    return "insufficient_imu_timing_evidence";
  }

  for (const auto& issue : snapshot.issues) {
    const std::string reason = causal_slam::diagnostics::ToString(issue.reason);
    if (reason != "none") {
      return reason;
    }
  }

  return causal_slam::policy::ToString(map_update_decision.reason);
}

causal_slam::statistics::CloudDecisionEvent MakeCloudDecisionEvent(const sensor_msgs::msg::PointCloud2& msg, std::int64_t receive_time_ns,
                                                                   std::uint64_t sequence_id,
                                                                   causal_slam::statistics::CloudForwardingDecision decision,
                                                                   const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                                                   const causal_slam::policy::MapUpdateDecision& map_update_decision) {
  causal_slam::statistics::CloudDecisionEvent event;
  event.sequence_id = sequence_id;
  event.header_stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  event.receive_time_ns = receive_time_ns;
  event.frame_id = msg.header.frame_id;
  event.point_count = static_cast<std::uint64_t>(msg.width) * static_cast<std::uint64_t>(msg.height);
  event.data_size_bytes = static_cast<std::uint64_t>(msg.data.size());
  event.health = snapshot.overall_status;
  event.map_update_allowed = map_update_decision.map_update_allowed;
  event.decision = decision;
  event.reason = PrimaryBlockReason(decision, snapshot, map_update_decision);
  event.fault_reasons = FaultReasonStrings(snapshot);

  if (snapshot.observation.lidar_scan_window.has_value()) {
    const auto& scan_window = *snapshot.observation.lidar_scan_window;
    event.has_scan_window = true;
    event.scan_duration_ms = scan_window.duration_ms;
    event.scan_window_source = causal_slam::lidar::ToString(scan_window.source);
    event.scan_window_confidence = causal_slam::lidar::ToString(scan_window.confidence);
  }

  if (snapshot.observation.imu_coverage.has_value()) {
    const auto& imu_coverage = *snapshot.observation.imu_coverage;
    event.has_imu_coverage = true;
    event.imu_samples_in_window = imu_coverage.imu_count_in_window;
    event.imu_coverage_ratio = imu_coverage.coverage_ratio;
    event.imu_max_gap_inside_ms = imu_coverage.max_gap_inside_ms;
  }

  return event;
}

}  // namespace

void TemporalMonitorNode::OnImuReceived(ImuMsg::ConstSharedPtr msg) {
  const std::int64_t header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

  if (!latest_imu_header_stamp_ns_.has_value() || header_stamp_ns > *latest_imu_header_stamp_ns_) {
    latest_imu_header_stamp_ns_ = header_stamp_ns;
  }

  temporal_pipeline_->ObserveImu(pipeline::ImuPipelineInput{
      .header_stamp_ns = header_stamp_ns,
      .receive_time_ns = this->now().nanoseconds(),
  });

  ProcessReadyPendingLidarClouds();
}

void TemporalMonitorNode::OnLidarReceived(PointCloud2Msg::ConstSharedPtr msg) {
  const std::int64_t header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const std::int64_t receive_time_ns = this->now().nanoseconds();

  if (!lidar_holdback_enabled_) {
    ProcessLidarCloud(msg, receive_time_ns, std::nullopt);
    return;
  }

  std::optional<causal_slam::lidar::LidarScanWindowEstimate> precomputed_scan_window_estimate;
  const std::int64_t ready_imu_stamp_ns = ComputeLidarHoldbackReadyStampNs(*msg, &precomputed_scan_window_estimate);

  if (latest_imu_header_stamp_ns_.has_value() && *latest_imu_header_stamp_ns_ >= ready_imu_stamp_ns) {
    ProcessLidarCloud(msg, receive_time_ns, precomputed_scan_window_estimate);
    return;
  }

  pending_lidar_clouds_.push_back(PendingLidarCloud{
      .msg = msg,
      .header_stamp_ns = header_stamp_ns,
      .receive_time_ns = receive_time_ns,
      .ready_imu_stamp_ns = ready_imu_stamp_ns,
      .precomputed_scan_window_estimate = precomputed_scan_window_estimate,
  });

  DropOldestPendingLidarCloudsIfNeeded();

  RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "LiDAR cloud held back"
                                   << " | stamp_ns=" << header_stamp_ns << " | frame_id=" << msg->header.frame_id
                                   << " | ready_imu_stamp_ns=" << ready_imu_stamp_ns << " | latest_imu_stamp_ns="
                                   << (latest_imu_header_stamp_ns_.has_value() ? std::to_string(*latest_imu_header_stamp_ns_)
                                                                               : std::string{"<none>"})
                                   << " | pending=" << pending_lidar_clouds_.size());
}

std::int64_t TemporalMonitorNode::ComputeLidarHoldbackReadyStampNs(
    const PointCloud2Msg& msg, std::optional<causal_slam::lidar::LidarScanWindowEstimate>* precomputed_scan_window_estimate) {
  if (precomputed_scan_window_estimate != nullptr) {
    precomputed_scan_window_estimate->reset();
  }

  const std::int64_t header_stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();

  const auto fields = ros_adapters::ToPointCloud2FieldInfos(msg);

  if (!holdback_point_time_field_.has_value()) {
    const auto inspection = holdback_point_cloud2_field_inspector_.Inspect(fields, holdback_point_time_config_);

    if (inspection.primary_time_field.has_value()) {
      holdback_point_time_field_ = *inspection.primary_time_field;
    }
  }

  if (holdback_point_time_field_.has_value()) {
    const auto extraction =
        holdback_point_cloud2_time_field_extractor_.Extract(ros_adapters::ToPointCloud2CloudView(msg), *holdback_point_time_field_);

    if (extraction.has_scan_window) {
      const auto estimate = BuildHoldbackPointTimeScanWindowEstimate(extraction);

      if (precomputed_scan_window_estimate != nullptr) {
        *precomputed_scan_window_estimate = estimate;
      }

      return estimate.window.end_ns + lidar_holdback_tolerance_ns_;
    }
  }

  const auto fallback_estimate = holdback_scan_window_estimator_.Estimate(header_stamp_ns);

  if (fallback_estimate.window.end_ns > header_stamp_ns) {
    return fallback_estimate.window.end_ns + lidar_holdback_tolerance_ns_;
  }

  return header_stamp_ns + lidar_holdback_scan_duration_ns_ + lidar_holdback_tolerance_ns_;
}

void TemporalMonitorNode::ProcessReadyPendingLidarClouds() {
  if (!latest_imu_header_stamp_ns_.has_value()) {
    return;
  }

  while (!pending_lidar_clouds_.empty() && *latest_imu_header_stamp_ns_ >= pending_lidar_clouds_.front().ready_imu_stamp_ns) {
    auto pending = pending_lidar_clouds_.front();
    pending_lidar_clouds_.pop_front();

    ProcessLidarCloud(pending.msg, pending.receive_time_ns, pending.precomputed_scan_window_estimate);
  }
}

void TemporalMonitorNode::DropOldestPendingLidarCloudsIfNeeded() {
  while (pending_lidar_clouds_.size() > lidar_holdback_max_pending_) {
    const auto pending = pending_lidar_clouds_.front();
    pending_lidar_clouds_.pop_front();

    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "LiDAR holdback queue full, dropping oldest pending cloud"
            << " | stamp_ns=" << pending.header_stamp_ns << " | ready_imu_stamp_ns=" << pending.ready_imu_stamp_ns
            << " | latest_imu_stamp_ns="
            << (latest_imu_header_stamp_ns_.has_value() ? std::to_string(*latest_imu_header_stamp_ns_) : std::string{"<none>"})
            << " | max_pending=" << lidar_holdback_max_pending_);
  }
}

void TemporalMonitorNode::ProcessLidarCloud(PointCloud2Msg::ConstSharedPtr msg, std::int64_t receive_time_ns,
                                            std::optional<causal_slam::lidar::LidarScanWindowEstimate> precomputed_scan_window_estimate) {
  const std::int64_t header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

  temporal_pipeline_->ObserveLidar(pipeline::LidarPipelineInput{
      .header_stamp_ns = header_stamp_ns,
      .receive_time_ns = receive_time_ns,
      .frame_id = msg->header.frame_id,
      .fields = ros_adapters::ToPointCloud2FieldInfos(*msg),
      .cloud_view = ros_adapters::ToPointCloud2CloudView(*msg),
      .precomputed_scan_window_estimate = precomputed_scan_window_estimate,
  });

  ObserveConfiguredTransformsForLidar(*msg, header_stamp_ns, receive_time_ns);

  const auto scan_snapshot = temporal_pipeline_->BuildLatestDiagnosticSnapshot();
  const auto scan_map_update_decision = causal_slam::policy::DecideMapUpdate(scan_snapshot.overall_status);

  PublishDiagnosticTopics(scan_snapshot, scan_map_update_decision);

  const auto forwarding_decision = MaybePublishCheckedLidar(*msg, scan_snapshot);

  temporal_pipeline_->ObserveCloudDecision(MakeCloudDecisionEvent(*msg, receive_time_ns, ++cloud_decision_sequence_id_, forwarding_decision,
                                                                  scan_snapshot, scan_map_update_decision));
}

void TemporalMonitorNode::ObserveConfiguredTransformsForLidar(const PointCloud2Msg& msg, std::int64_t header_stamp_ns,
                                                              std::int64_t receive_time_ns) {
  if (!tf_monitoring_enabled_ || tf_buffer_ == nullptr) {
    return;
  }

  std::vector<causal_slam::transform::TransformLookupObservation> observations;
  observations.reserve(transform_checks_.size());

  for (const auto& check : transform_checks_) {
    const std::string source_frame = ResolveSourceFrame(check.source_frame, msg.header.frame_id);

    observations.push_back(ros_adapters::LookupTransform(*tf_buffer_, ros_adapters::TransformLookupRequest{
                                                                          .target_frame = check.target_frame,
                                                                          .source_frame = source_frame,
                                                                          .requested_stamp_ns = header_stamp_ns,
                                                                          .receive_time_ns = receive_time_ns,
                                                                      }));
  }

  temporal_pipeline_->ObserveTransforms(observations);
}

causal_slam::statistics::CloudForwardingDecision TemporalMonitorNode::MaybePublishCheckedLidar(
    const PointCloud2Msg& msg, const diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  if (!checked_lidar_publisher_) {
    return causal_slam::statistics::CloudForwardingDecision::kBlockedByGate;
  }

  const auto gate_result = policy::EvaluateLidarCloudGate(lidar_gate_config_, BuildLidarCloudGateInput(snapshot));

  if (!gate_result.should_forward) {
    if (gate_result.reason == policy::LidarCloudGateReason::kInsufficientTimingEvidence) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "LiDAR cloud blocked by temporal gate warmup"
                                      << " | stamp_ns=" << snapshot.observation.latest_lidar_header_stamp_ns
                                      << " | frame_id=" << snapshot.observation.latest_lidar_frame_id
                                      << " | gate_mode=" << policy::ToString(lidar_gate_config_.mode)
                                      << " | gate_reason=" << policy::ToString(gate_result.reason));

      return causal_slam::statistics::CloudForwardingDecision::kBlockedWarmup;
    }

    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "LiDAR cloud blocked by temporal gate"
                                    << " | stamp_ns=" << snapshot.observation.latest_lidar_header_stamp_ns
                                    << " | frame_id=" << snapshot.observation.latest_lidar_frame_id
                                    << " | health=" << telemetry::ToString(snapshot.overall_status)
                                    << " | gate_mode=" << policy::ToString(lidar_gate_config_.mode)
                                    << " | gate_reason=" << policy::ToString(gate_result.reason)
                                    << " | reasons=" << diagnostics::JoinFaultReasons(snapshot.issues));

    return causal_slam::statistics::CloudForwardingDecision::kBlockedByGate;
  }

  checked_lidar_publisher_->publish(msg);
  return causal_slam::statistics::CloudForwardingDecision::kForwarded;
}

void TemporalMonitorNode::PublishDiagnosticTopics(const diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                                  const policy::MapUpdateDecision& map_update_decision) {
  BoolMsg map_update_allowed_msg;
  map_update_allowed_msg.data = map_update_decision.map_update_allowed;
  map_update_allowed_publisher_->publish(map_update_allowed_msg);

  StringMsg temporal_health_msg;
  temporal_health_msg.data = telemetry::ToString(snapshot.overall_status);
  temporal_health_publisher_->publish(temporal_health_msg);

  StringMsg map_update_reason_msg;
  map_update_reason_msg.data = causal_slam::policy::ToString(map_update_decision.reason);

  map_update_reason_publisher_->publish(map_update_reason_msg);

  StringMsg fault_reasons_msg;
  fault_reasons_msg.data = diagnostics::JoinFaultReasons(snapshot.issues);
  fault_reasons_publisher_->publish(fault_reasons_msg);

  StringMsg decision_json_msg;
  decision_json_msg.data = render::RenderMapUpdateDecisionJson(snapshot, map_update_decision);
  map_update_decision_json_publisher_->publish(decision_json_msg);
}

void TemporalMonitorNode::OnTimer() {
  if (runtime_profile_ == RuntimeProfile::kMinimal) {
    return;
  }

  const std::int64_t now_ns = this->now().nanoseconds();
  const auto snapshot = temporal_pipeline_->BuildSnapshot(now_ns);

  PublishDiagnosticTopics(snapshot.diagnostics, snapshot.map_update_decision);

  if (runtime_profile_ == RuntimeProfile::kDiagnostic) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Temporal gate summary"
                                               << " | profile=" << ToString(runtime_profile_)
                                               << " | health=" << telemetry::ToString(snapshot.diagnostics.overall_status)
                                               << " | allowed=" << (snapshot.map_update_decision.map_update_allowed ? "true" : "false")
                                               << " | reason=" << causal_slam::policy::ToString(snapshot.map_update_decision.reason)
                                               << " | fault_reasons=" << diagnostics::JoinFaultReasons(snapshot.diagnostics.issues));
    return;
  }

  for (const auto& stream : snapshot.diagnostics.observation.streams) {
    LogTimingSummary(this->get_logger(), stream);
  }

  LogImuCoverageSummary(this->get_logger(), snapshot.diagnostics.observation.imu_coverage,
                        snapshot.diagnostics.observation.lidar_scan_window, snapshot.diagnostics.observation.imu_buffer_size);

  const render::ConsoleTemporalSummaryRenderer renderer;
  RCLCPP_INFO_STREAM(this->get_logger(), "\n" << renderer.Render(snapshot.diagnostics, snapshot.map_update_decision) << '\n'
                                              << renderer.RenderStatistics(snapshot.statistics));

  if (!html_report_path_.empty()) {
    const render::HtmlTemporalSummaryRenderer html_renderer;
    const auto result = platform::WriteTextFileAtomically(
        html_report_path_, html_renderer.RenderPage(snapshot.diagnostics, snapshot.map_update_decision, snapshot.statistics));
    if (!result.ok) {
      RCLCPP_WARN(this->get_logger(), "Failed to write HTML temporal report | path=%s | error=%s", html_report_path_.c_str(),
                  result.error.c_str());
    }
  }
}

}  // namespace causal_slam::nodes
