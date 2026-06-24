#include "temporal_monitor_node.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "config/temporal_monitor_runtime_defaults.h"
#include "coverage/imu_coverage_analyzer.h"
#include "diagnostics/temporal_fault_reason_formatter.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "model/temporal_observation.h"
#include "platform/atomic_file_writer.h"
#include "policy/map_update_decision.h"
#include "render/console_temporal_summary_renderer.h"
#include "render/html_temporal_summary_renderer.h"
#include "render/map_update_decision_json_renderer.h"
#include "ros_adapters/point_cloud2_conversions.h"
#include "ros_adapters/transform_lookup_adapter.h"

namespace causal_slam::nodes {

namespace config = causal_slam::config;
namespace coverage = causal_slam::coverage;
namespace diagnostics = causal_slam::diagnostics;
namespace lidar = causal_slam::lidar;
namespace pipeline = causal_slam::pipeline;
namespace platform = causal_slam::platform;
namespace render = causal_slam::render;
namespace ros_adapters = causal_slam::ros_adapters;
namespace statistics = causal_slam::statistics;
namespace telemetry = causal_slam::telemetry;

namespace {

void LogTimingSummary(
    const rclcpp::Logger& logger,
    const telemetry::StreamTimingDiagnostic& stream) {
  const auto& summary = stream.timing;

  RCLCPP_INFO_STREAM(
      logger, telemetry::ToString(stream.id)
                  << " timing"
                  << " | total_count=" << summary.total_count
                  << " | window_count=" << summary.window_count
                  << " | last_delay_ms=" << summary.last_delay_ms
                  << " | window_avg_delay_ms="
                  << summary.window_average_delay_ms
                  << " | window_max_delay_ms=" << summary.window_max_delay_ms
                  << " | last_period_ms=" << summary.last_period_ms
                  << " | last_jitter_ms=" << summary.last_jitter_ms
                  << " | window_max_jitter_ms="
                  << summary.window_max_jitter_ms
                  << " | total_reordered_count="
                  << summary.total_reordered_count
                  << " | window_reordered_count="
                  << summary.window_reordered_count
                  << " | total_gap_count=" << summary.total_gap_count
                  << " | window_gap_count=" << summary.window_gap_count
                  << " | max_gap_ms=" << summary.max_gap_ms
                  << " | health=" << telemetry::ToString(summary.health)
                  << " | reason=" << summary.reason);
}

RuntimeProfile ParseRuntimeProfile(std::string_view value) {
  if (value == "minimal") {
    return RuntimeProfile::kMinimal;
  }

  if (value == "diagnostic") {
    return RuntimeProfile::kDiagnostic;
  }

  if (value == "debug_report") {
    return RuntimeProfile::kDebugReport;
  }

  // Preserve old verbose behavior for unknown values.
  return RuntimeProfile::kDebugReport;
}

const char* ToString(RuntimeProfile profile) {
  switch (profile) {
    case RuntimeProfile::kMinimal:
      return "minimal";
    case RuntimeProfile::kDiagnostic:
      return "diagnostic";
    case RuntimeProfile::kDebugReport:
      return "debug_report";
  }

  return "debug_report";
}

lidar::LidarStampPolicy ParseLidarStampPolicy(std::string_view value) {
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

std::int64_t MillisecondsToNanoseconds(double milliseconds) {
  constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

  const double safe_milliseconds = std::max(milliseconds, 0.0);
  return static_cast<std::int64_t>(
      safe_milliseconds * kNanosecondsPerMillisecond);
}

std::string ResolveSourceFrame(
    std::string configured_source_frame,
    const std::string& cloud_frame_id) {
  if (configured_source_frame.empty() ||
      configured_source_frame == "<cloud_frame>" ||
      configured_source_frame == "cloud_frame") {
    return cloud_frame_id;
  }

  return configured_source_frame;
}

bool IsActiveLidarGateMode(std::string_view gate_mode) {
  return gate_mode != "observe";
}

bool HasMinimumTimingEvidenceForActiveGate(
    const diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  constexpr std::uint64_t kMinTotalImuSamplesBeforeForward = 5;
  constexpr std::uint64_t kMinWindowImuSamplesBeforeForward = 2;

  for (const auto& stream : snapshot.observation.streams) {
    if (stream.id != telemetry::TemporalStreamId::kImu) {
      continue;
    }

    return stream.timing.total_count >= kMinTotalImuSamplesBeforeForward &&
           stream.timing.window_count >= kMinWindowImuSamplesBeforeForward;
  }

  return false;
}

bool ShouldForwardLidarCloud(
    std::string_view gate_mode,
    causal_slam::telemetry::TemporalHealthStatus health) {
  using causal_slam::telemetry::TemporalHealthStatus;

  if (gate_mode == "observe") {
    return true;
  }

  if (gate_mode == "drop_invalid") {
    return health != TemporalHealthStatus::kInvalid;
  }

  if (gate_mode == "drop_degraded") {
    return health == TemporalHealthStatus::kOk ||
           health == TemporalHealthStatus::kWarning;
  }

  if (gate_mode == "strict") {
    return health == TemporalHealthStatus::kOk;
  }

  // Unknown mode is intentionally safe for existing deployments:
  // monitor-only behavior, no surprise cloud drops.
  return true;
}

std::vector<TransformCheckConfig> BuildTransformChecks(
    const std::vector<std::string>& target_frames,
    const std::vector<std::string>& source_frames) {
  std::vector<TransformCheckConfig> checks;

  if (target_frames.size() != source_frames.size()) {
    return checks;
  }

  checks.reserve(target_frames.size());

  for (std::size_t i = 0; i < target_frames.size(); ++i) {
    checks.push_back(TransformCheckConfig{
        .target_frame = target_frames[i],
        .source_frame = source_frames[i],
    });
  }

  return checks;
}

void LogImuCoverageSummary(
    const rclcpp::Logger& logger,
    const std::optional<coverage::ImuCoverageSummary>& summary,
    const std::optional<lidar::LidarScanWindowEstimate>& scan_window_estimate,
    std::size_t imu_buffer_size) {
  if (!summary.has_value() || !scan_window_estimate.has_value()) {
    RCLCPP_INFO_STREAM(logger, "IMU coverage"
                                   << " | status=not_available"
                                   << " | reason=no_lidar_scan_received_yet"
                                   << " | imu_buffer_size=" << imu_buffer_size);
    return;
  }

  const auto& coverage_summary = *summary;
  const auto& scan_window = *scan_window_estimate;

  RCLCPP_INFO_STREAM(
      logger, "IMU coverage"
                  << " | imu_count_in_window="
                  << coverage_summary.imu_count_in_window
                  << " | missing_prefix_ms="
                  << coverage_summary.missing_prefix_ms
                  << " | missing_suffix_ms="
                  << coverage_summary.missing_suffix_ms
                  << " | max_gap_inside_ms="
                  << coverage_summary.max_gap_inside_ms
                  << " | coverage_ratio=" << coverage_summary.coverage_ratio
                  << " | health=" << coverage::ToString(coverage_summary.health)
                  << " | reason=" << coverage_summary.reason
                  << " | scan_window_duration_ms=" << scan_window.duration_ms
                  << " | scan_window_source="
                  << lidar::ToString(scan_window.source)
                  << " | scan_window_confidence="
                  << lidar::ToString(scan_window.confidence)
                  << " | scan_window_reason=" << scan_window.reason
                  << " | imu_buffer_size=" << imu_buffer_size);
}

}  // namespace

TemporalMonitorNode::TemporalMonitorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("temporal_monitor_node", options) {
  const auto defaults = config::MakeDefaultTemporalMonitorRuntimeDefaults();

  runtime_profile_ = ParseRuntimeProfile(
      this->declare_parameter<std::string>("runtime_profile", "debug_report"));

  const std::string imu_topic =
      this->declare_parameter<std::string>("imu_topic", defaults.imu_topic);
  const std::string lidar_topic =
      this->declare_parameter<std::string>("lidar_topic", defaults.lidar_topic);
  const std::string checked_lidar_topic =
      this->declare_parameter<std::string>(
          "checked_lidar_topic", defaults.checked_lidar_topic);
  lidar_gate_mode_ =
      this->declare_parameter<std::string>(
          "lidar_gate_mode", defaults.lidar_gate_mode);

  const std::string map_update_allowed_topic =
      this->declare_parameter<std::string>(
          "map_update_allowed_topic", defaults.map_update_allowed_topic);
  const std::string temporal_health_topic =
      this->declare_parameter<std::string>(
          "temporal_health_topic", defaults.temporal_health_topic);
  const std::string map_update_reason_topic =
      this->declare_parameter<std::string>(
          "map_update_reason_topic", defaults.map_update_reason_topic);
  const std::string fault_reasons_topic =
      this->declare_parameter<std::string>(
          "fault_reasons_topic", defaults.fault_reasons_topic);
  const std::string map_update_decision_json_topic =
      this->declare_parameter<std::string>(
          "map_update_decision_json_topic",
          defaults.map_update_decision_json_topic);

  html_report_path_ =
      this->declare_parameter<std::string>(
          "html_report_path", defaults.html_report_path);

  const double summary_period_ms =
      this->declare_parameter<double>("summary_period_ms", defaults.summary_period_ms);
  const double safe_summary_period_ms = std::max(summary_period_ms, defaults.limits.min_summary_period_ms);

  const double imu_gap_threshold_ms =
      this->declare_parameter<double>("imu_gap_threshold_ms", defaults.pipeline.imu_gap_threshold_ms);
  const double safe_imu_gap_threshold_ms =
      std::max(imu_gap_threshold_ms, defaults.limits.min_gap_threshold_ms);

  const double lidar_gap_threshold_ms =
      this->declare_parameter<double>("lidar_gap_threshold_ms", defaults.pipeline.lidar_gap_threshold_ms);
  const double safe_lidar_gap_threshold_ms =
      std::max(lidar_gap_threshold_ms, defaults.limits.min_gap_threshold_ms);

  const double lidar_scan_duration_ms =
      this->declare_parameter<double>("lidar_scan_duration_ms", defaults.pipeline.lidar_scan_window.fallback_scan_duration_ms);
  const double safe_lidar_scan_duration_ms =
      std::max(lidar_scan_duration_ms, defaults.limits.min_lidar_scan_duration_ms);

  const double lidar_min_measured_scan_duration_ms =
      this->declare_parameter<double>(
          "lidar_min_measured_scan_duration_ms", defaults.pipeline.lidar_scan_window.min_measured_scan_duration_ms);
  const double safe_lidar_min_measured_scan_duration_ms =
      std::max(lidar_min_measured_scan_duration_ms, defaults.limits.min_lidar_measured_scan_duration_ms);

  const double lidar_max_measured_scan_duration_ms =
      this->declare_parameter<double>(
          "lidar_max_measured_scan_duration_ms", defaults.pipeline.lidar_scan_window.max_measured_scan_duration_ms);
  const double safe_lidar_max_measured_scan_duration_ms =
      std::max(lidar_max_measured_scan_duration_ms,
               safe_lidar_min_measured_scan_duration_ms);

  const bool lidar_prefer_measured_header_period =
      this->declare_parameter<bool>("lidar_prefer_measured_header_period", defaults.pipeline.lidar_scan_window.prefer_measured_header_period);

  const std::string lidar_stamp_policy =
      this->declare_parameter<std::string>("lidar_stamp_policy", std::string{lidar::ToString(defaults.pipeline.lidar_scan_window.stamp_policy)});
  const auto parsed_lidar_stamp_policy =
      ParseLidarStampPolicy(lidar_stamp_policy);

  const double imu_buffer_retention_ms =
      this->declare_parameter<double>("imu_buffer_retention_ms", defaults.imu_buffer_retention_ms);
  const double safe_imu_buffer_retention_ms =
      std::max(imu_buffer_retention_ms, defaults.limits.min_imu_buffer_retention_ms);

  const double expected_imu_period_ms =
      this->declare_parameter<double>("expected_imu_period_ms", defaults.expected_imu_period_ms);
  const double safe_expected_imu_period_ms =
      std::max(expected_imu_period_ms, defaults.limits.min_expected_imu_period_ms);

  const double max_missing_prefix_ms =
      this->declare_parameter<double>(
          "max_missing_prefix_ms", defaults.limits.imu_coverage_missing_prefix_periods * safe_expected_imu_period_ms);
  const double max_missing_suffix_ms =
      this->declare_parameter<double>(
          "max_missing_suffix_ms", defaults.limits.imu_coverage_missing_suffix_periods * safe_expected_imu_period_ms);
  const double max_internal_gap_ms =
      this->declare_parameter<double>(
          "max_internal_gap_ms", defaults.limits.imu_coverage_max_internal_gap_periods * safe_expected_imu_period_ms);

  tf_monitoring_enabled_ =
      this->declare_parameter<bool>("tf_monitoring_enabled", defaults.tf_monitoring_enabled);

  const std::vector<std::string> tf_target_frames =
      this->declare_parameter<std::vector<std::string>>(
          "tf_target_frames", defaults.tf_target_frames);
  const std::vector<std::string> tf_source_frames =
      this->declare_parameter<std::vector<std::string>>(
          "tf_source_frames", defaults.tf_source_frames);

  const double tf_max_transform_age_ms =
      this->declare_parameter<double>("tf_max_transform_age_ms", defaults.pipeline.transform_age.max_transform_age_ms);
  const double tf_max_future_tolerance_ms =
      this->declare_parameter<double>("tf_max_future_tolerance_ms", defaults.pipeline.transform_age.max_future_tolerance_ms);

  auto pipeline_config = pipeline::TemporalMonitorPipelineConfig{};
  pipeline_config.imu_gap_threshold_ms = safe_imu_gap_threshold_ms;
  pipeline_config.lidar_gap_threshold_ms = safe_lidar_gap_threshold_ms;
  pipeline_config.imu_buffer_retention_ns =
      MillisecondsToNanoseconds(safe_imu_buffer_retention_ms);
  pipeline_config.lidar_scan_window =
      lidar::LidarScanWindowEstimatorConfig{
          .fallback_scan_duration_ms = safe_lidar_scan_duration_ms,
          .min_measured_scan_duration_ms =
              safe_lidar_min_measured_scan_duration_ms,
          .max_measured_scan_duration_ms =
              safe_lidar_max_measured_scan_duration_ms,
          .stamp_policy = parsed_lidar_stamp_policy,
          .prefer_measured_header_period =
              lidar_prefer_measured_header_period,
      };
  pipeline_config.imu_coverage = coverage::ImuCoverageConfig{
      .max_missing_prefix_ms = std::max(max_missing_prefix_ms, 0.0),
      .max_missing_suffix_ms = std::max(max_missing_suffix_ms, 0.0),
      .max_internal_gap_ms =
          std::max(max_internal_gap_ms, safe_expected_imu_period_ms),
  };

  pipeline_config.transform_age.max_transform_age_ms =
      std::max(tf_max_transform_age_ms, 0.0);
  pipeline_config.transform_age.max_future_tolerance_ms =
      std::max(tf_max_future_tolerance_ms, 0.0);

  temporal_pipeline_.emplace(pipeline_config);

  transform_checks_ = BuildTransformChecks(tf_target_frames, tf_source_frames);

  if (tf_monitoring_enabled_ && transform_checks_.empty()) {
    RCLCPP_WARN(
        this->get_logger(),
        "TF monitoring requested, but tf_target_frames/tf_source_frames are "
        "empty or have different sizes. TF monitoring is disabled.");
    tf_monitoring_enabled_ = false;
  }

  if (tf_monitoring_enabled_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }

  imu_subscription_ =
      this->create_subscription<ImuMsg>(
          imu_topic, rclcpp::SensorDataQoS{},
          [this](ImuMsg::ConstSharedPtr msg) { OnImuReceived(msg); });

  lidar_subscription_ =
      this->create_subscription<PointCloud2Msg>(
          lidar_topic, rclcpp::SensorDataQoS{},
          [this](PointCloud2Msg::ConstSharedPtr msg) { OnLidarReceived(msg); });

  auto diagnostic_qos = rclcpp::QoS{1};
  diagnostic_qos.reliable();
  diagnostic_qos.transient_local();

  map_update_allowed_publisher_ =
      this->create_publisher<BoolMsg>(
          map_update_allowed_topic, diagnostic_qos);
  temporal_health_publisher_ =
      this->create_publisher<StringMsg>(
          temporal_health_topic, diagnostic_qos);
  map_update_reason_publisher_ =
      this->create_publisher<StringMsg>(
          map_update_reason_topic, diagnostic_qos);
  fault_reasons_publisher_ =
      this->create_publisher<StringMsg>(
          fault_reasons_topic, diagnostic_qos);
  map_update_decision_json_publisher_ =
      this->create_publisher<StringMsg>(
          map_update_decision_json_topic, diagnostic_qos);

  if (!checked_lidar_topic.empty()) {
    checked_lidar_publisher_ =
        this->create_publisher<PointCloud2Msg>(
            checked_lidar_topic, rclcpp::SensorDataQoS{});
  }

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double, std::milli>(safe_summary_period_ms)),
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
              imu_topic.c_str(), lidar_topic.c_str(),
              checked_lidar_topic.empty() ? "<disabled>" : checked_lidar_topic.c_str(),
              lidar_gate_mode_.c_str(),
              map_update_allowed_topic.c_str(), temporal_health_topic.c_str(),
              map_update_reason_topic.c_str(), fault_reasons_topic.c_str(),
              safe_summary_period_ms,
              safe_imu_gap_threshold_ms, safe_lidar_gap_threshold_ms,
              safe_lidar_scan_duration_ms,
              safe_lidar_min_measured_scan_duration_ms,
              safe_lidar_max_measured_scan_duration_ms,
              lidar_prefer_measured_header_period ? "true" : "false",
              lidar::ToString(parsed_lidar_stamp_policy),
              safe_expected_imu_period_ms, safe_imu_buffer_retention_ms,
              std::max(max_missing_prefix_ms, 0.0),
              std::max(max_missing_suffix_ms, 0.0),
              std::max(max_internal_gap_ms, safe_expected_imu_period_ms));


  RCLCPP_INFO(this->get_logger(),
              "TF monitoring"
              " | enabled=%s"
              " | check_count=%zu"
              " | tf_max_transform_age_ms=%.3f"
              " | tf_max_future_tolerance_ms=%.3f",
              tf_monitoring_enabled_ ? "true" : "false",
              transform_checks_.size(),
              std::max(tf_max_transform_age_ms, 0.0),
              std::max(tf_max_future_tolerance_ms, 0.0));


  RCLCPP_INFO(this->get_logger(),
              "HTML report"
              " | enabled=%s"
              " | path=%s",
              html_report_path_.empty() ? "false" : "true",
              html_report_path_.c_str());
}

void TemporalMonitorNode::OnImuReceived(ImuMsg::ConstSharedPtr msg) {
  temporal_pipeline_->ObserveImu(pipeline::ImuPipelineInput{
      .header_stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds(),
      .receive_time_ns = this->now().nanoseconds(),
  });
}

void TemporalMonitorNode::OnLidarReceived(PointCloud2Msg::ConstSharedPtr msg) {
  const std::int64_t header_stamp_ns =
      rclcpp::Time(msg->header.stamp).nanoseconds();
  const std::int64_t receive_time_ns = this->now().nanoseconds();

  temporal_pipeline_->ObserveLidar(pipeline::LidarPipelineInput{
      .header_stamp_ns = header_stamp_ns,
      .receive_time_ns = receive_time_ns,
      .frame_id = msg->header.frame_id,
      .fields = ros_adapters::ToPointCloud2FieldInfos(*msg),
      .cloud_view = ros_adapters::ToPointCloud2CloudView(*msg),
  });

  ObserveConfiguredTransformsForLidar(*msg, header_stamp_ns, receive_time_ns);

  const auto scan_snapshot = temporal_pipeline_->BuildLatestDiagnosticSnapshot();
  PublishDiagnosticTopics(scan_snapshot);
  MaybePublishCheckedLidar(*msg, scan_snapshot);
}

void TemporalMonitorNode::ObserveConfiguredTransformsForLidar(
    const PointCloud2Msg& msg,
    std::int64_t header_stamp_ns,
    std::int64_t receive_time_ns) {
  if (!tf_monitoring_enabled_ || tf_buffer_ == nullptr) {
    return;
  }

  std::vector<causal_slam::transform::TransformLookupObservation> observations;
  observations.reserve(transform_checks_.size());

  for (const auto& check : transform_checks_) {
    const std::string source_frame =
        ResolveSourceFrame(check.source_frame, msg.header.frame_id);

    observations.push_back(
        ros_adapters::LookupTransform(
            *tf_buffer_,
            ros_adapters::TransformLookupRequest{
                .target_frame = check.target_frame,
                .source_frame = source_frame,
                .requested_stamp_ns = header_stamp_ns,
                .receive_time_ns = receive_time_ns,
            }));
  }

  temporal_pipeline_->ObserveTransforms(observations);
}

void TemporalMonitorNode::MaybePublishCheckedLidar(
    const PointCloud2Msg& msg,
    const diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  if (!checked_lidar_publisher_) {
    return;
  }

  if (IsActiveLidarGateMode(lidar_gate_mode_) &&
      !HasMinimumTimingEvidenceForActiveGate(snapshot)) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "LiDAR cloud blocked by temporal gate warmup"
            << " | stamp_ns="
            << snapshot.observation.latest_lidar_header_stamp_ns
            << " | frame_id=" << snapshot.observation.latest_lidar_frame_id
            << " | reason=insufficient_imu_timing_evidence");
    return;
  }

  if (!ShouldForwardLidarCloud(lidar_gate_mode_, snapshot.overall_status)) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "LiDAR cloud blocked by temporal gate"
            << " | stamp_ns="
            << snapshot.observation.latest_lidar_header_stamp_ns
            << " | frame_id=" << snapshot.observation.latest_lidar_frame_id
            << " | health=" << telemetry::ToString(snapshot.overall_status)
            << " | reasons=" << diagnostics::JoinFaultReasons(snapshot.issues));
    return;
  }

  checked_lidar_publisher_->publish(msg);
}

void TemporalMonitorNode::PublishDiagnosticTopics(
    const diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  BoolMsg map_update_allowed_msg;
  map_update_allowed_msg.data =
      snapshot.map_update_decision.map_update_allowed;
  map_update_allowed_publisher_->publish(map_update_allowed_msg);

  StringMsg temporal_health_msg;
  temporal_health_msg.data = telemetry::ToString(snapshot.overall_status);
  temporal_health_publisher_->publish(temporal_health_msg);

  StringMsg map_update_reason_msg;
  map_update_reason_msg.data =
      causal_slam::policy::ToString(snapshot.map_update_decision.reason);
  map_update_reason_publisher_->publish(map_update_reason_msg);

  StringMsg fault_reasons_msg;
  fault_reasons_msg.data = diagnostics::JoinFaultReasons(snapshot.issues);
  fault_reasons_publisher_->publish(fault_reasons_msg);

  StringMsg decision_json_msg;
  decision_json_msg.data = render::RenderMapUpdateDecisionJson(snapshot);
  map_update_decision_json_publisher_->publish(decision_json_msg);
}

void TemporalMonitorNode::OnTimer() {
  if (runtime_profile_ == RuntimeProfile::kMinimal) {
    return;
  }

  const std::int64_t now_ns = this->now().nanoseconds();
  const auto snapshot = temporal_pipeline_->BuildSnapshot(now_ns);

  PublishDiagnosticTopics(snapshot.diagnostics);

  if (runtime_profile_ == RuntimeProfile::kDiagnostic) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Temporal gate summary"
            << " | profile=" << ToString(runtime_profile_)
            << " | health="
            << telemetry::ToString(snapshot.diagnostics.overall_status)
            << " | allowed="
            << (snapshot.diagnostics.map_update_decision.map_update_allowed
                    ? "true"
                    : "false")
            << " | reason="
            << causal_slam::policy::ToString(
                   snapshot.diagnostics.map_update_decision.reason)
            << " | fault_reasons="
            << diagnostics::JoinFaultReasons(snapshot.diagnostics.issues));
    return;
  }

  for (const auto& stream : snapshot.diagnostics.observation.streams) {
    LogTimingSummary(this->get_logger(), stream);
  }

  LogImuCoverageSummary(
      this->get_logger(),
      snapshot.diagnostics.observation.imu_coverage,
      snapshot.diagnostics.observation.lidar_scan_window,
      snapshot.diagnostics.observation.imu_buffer_size);

  const render::ConsoleTemporalSummaryRenderer renderer;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "\n" << renderer.Render(snapshot.diagnostics) << '\n'
                           << renderer.RenderStatistics(snapshot.statistics));

  if (!html_report_path_.empty()) {
    const render::HtmlTemporalSummaryRenderer html_renderer;
    const auto result = platform::WriteTextFileAtomically(
        html_report_path_,
        html_renderer.RenderPage(snapshot.diagnostics, snapshot.statistics));

    if (!result.ok) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to write HTML temporal report | path=%s | error=%s",
                  html_report_path_.c_str(),
                  result.error.c_str());
    }
  }
}

}  // namespace causal_slam::nodes
