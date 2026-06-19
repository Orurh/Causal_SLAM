#include "temporal_monitor_node.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>

#include "render/console_temporal_summary_renderer.h"

namespace causal_slam::nodes {

namespace coverage = causal_slam::coverage;
namespace diagnostics = causal_slam::diagnostics;
namespace lidar = causal_slam::lidar;
namespace render = causal_slam::render;
namespace ros_adapters = causal_slam::ros_adapters;
namespace telemetry = causal_slam::telemetry;

namespace {

void LogTimingSummary(const rclcpp::Logger& logger, const char* stream_name, const telemetry::TimingSummary& summary) {
  RCLCPP_INFO_STREAM(
      logger, stream_name << " timing"
                          << " | total_count=" << summary.total_count << " | window_count=" << summary.window_count
                          << " | last_delay_ms=" << summary.last_delay_ms << " | window_avg_delay_ms=" << summary.window_average_delay_ms
                          << " | window_max_delay_ms=" << summary.window_max_delay_ms << " | last_period_ms=" << summary.last_period_ms
                          << " | last_jitter_ms=" << summary.last_jitter_ms << " | window_max_jitter_ms=" << summary.window_max_jitter_ms
                          << " | total_reordered_count=" << summary.total_reordered_count << " | window_reordered_count="
                          << summary.window_reordered_count << " | total_gap_count=" << summary.total_gap_count
                          << " | window_gap_count=" << summary.window_gap_count << " | max_gap_ms=" << summary.max_gap_ms
                          << " | health=" << telemetry::ToString(summary.health) << " | reason=" << summary.reason);
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
  return static_cast<std::int64_t>(safe_milliseconds * kNanosecondsPerMillisecond);
}

double NanosecondsToMilliseconds(std::int64_t nanoseconds) {
  constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

  return static_cast<double>(nanoseconds) / kNanosecondsPerMillisecond;
}

lidar::LidarScanWindowEstimate BuildPointTimeFieldEstimate(const ros_adapters::PointCloud2TimeFieldExtraction& extraction) {
  return lidar::LidarScanWindowEstimate{
      .window = extraction.scan_window,
      .duration_ms = NanosecondsToMilliseconds(extraction.scan_window.DurationNs()),
      .source = lidar::LidarScanWindowSource::kPointTimeField,
      .confidence = lidar::LidarScanWindowConfidence::kHigh,
      .reason = extraction.reason + ":" + ros_adapters::ToString(extraction.time_unit),
  };
}

void LogImuCoverageSummary(const rclcpp::Logger& logger, bool has_summary, const coverage::ImuCoverageSummary& summary,
                           const lidar::LidarScanWindowEstimate& scan_window_estimate, std::size_t imu_buffer_size) {
  if (!has_summary) {
    RCLCPP_INFO_STREAM(logger, "IMU coverage" << " | status=not_available"
                                              << " | reason=no_lidar_scan_received_yet"
                                              << " | imu_buffer_size=" << imu_buffer_size);
    return;
  }

  RCLCPP_INFO_STREAM(
      logger, "IMU coverage" << " | imu_count_in_window=" << summary.imu_count_in_window
                             << " | missing_prefix_ms=" << summary.missing_prefix_ms << " | missing_suffix_ms=" << summary.missing_suffix_ms
                             << " | max_gap_inside_ms=" << summary.max_gap_inside_ms << " | coverage_ratio=" << summary.coverage_ratio
                             << " | health=" << coverage::ToString(summary.health) << " | reason=" << summary.reason
                             << " | scan_window_duration_ms=" << scan_window_estimate.duration_ms
                             << " | scan_window_source=" << lidar::ToString(scan_window_estimate.source)
                             << " | scan_window_confidence=" << lidar::ToString(scan_window_estimate.confidence)
                             << " | scan_window_reason=" << scan_window_estimate.reason << " | imu_buffer_size=" << imu_buffer_size);
}

void LogPointCloud2FieldInspection(const rclcpp::Logger& logger, const ros_adapters::PointCloud2FieldInspection& inspection) {
  std::ostringstream fields_stream;

  for (std::size_t i = 0; i < inspection.fields.size(); ++i) {
    const auto& field = inspection.fields[i];

    if (i > 0) {
      fields_stream << ", ";
    }

    fields_stream << field.name << ":" << ros_adapters::PointCloud2DatatypeToString(field.datatype) << "@offset=" << field.offset
                  << ":count=" << field.count;

    if (field.time_role != ros_adapters::PointCloud2TimeFieldRole::kNone) {
      fields_stream << ":time_role=" << ros_adapters::ToString(field.time_role);
    }
  }

  RCLCPP_INFO_STREAM(logger, "LiDAR PointCloud2 fields"
                                 << " | field_count=" << inspection.fields.size()
                                 << " | has_time_candidate=" << (inspection.has_time_candidate ? "true" : "false")
                                 << " | has_supported_time_field=" << (inspection.has_supported_time_field ? "true" : "false")
                                 << " | reason=" << inspection.reason << " | fields=[" << fields_stream.str() << "]");

  if (inspection.primary_time_field.has_value()) {
    const auto& field = *inspection.primary_time_field;

    RCLCPP_INFO_STREAM(
        logger, "LiDAR PointCloud2 primary time field"
                    << " | name=" << field.name << " | datatype=" << ros_adapters::PointCloud2DatatypeToString(field.datatype)
                    << " | offset=" << field.offset << " | count=" << field.count << " | role=" << ros_adapters::ToString(field.time_role));
  }
}

diagnostics::PointTimeDiagnostics BuildPointTimeDiagnostics(const ros_adapters::PointCloud2FieldInspection& inspection) {
  diagnostics::PointTimeDiagnostics diagnostics;

  diagnostics.has_time_candidate = inspection.has_time_candidate;
  diagnostics.has_supported_time_field = inspection.has_supported_time_field;
  diagnostics.inspection_reason = inspection.reason;

  const ros_adapters::PointCloud2FieldInfo* diagnostic_field = nullptr;

  if (inspection.primary_time_field.has_value()) {
    diagnostic_field = &*inspection.primary_time_field;
  } else {
    for (const auto& field : inspection.fields) {
      if (field.time_role != ros_adapters::PointCloud2TimeFieldRole::kNone) {
        diagnostic_field = &field;
        break;
      }
    }
  }

  if (diagnostic_field != nullptr) {
    diagnostics.field_name = diagnostic_field->name;
    diagnostics.field_datatype = ros_adapters::PointCloud2DatatypeToString(diagnostic_field->datatype);
    diagnostics.field_role = ros_adapters::ToString(diagnostic_field->time_role);
  }

  return diagnostics;
}

}  // namespace

TemporalMonitorNode::TemporalMonitorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("temporal_monitor_node", options) {
  const std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  const std::string lidar_topic = this->declare_parameter<std::string>("lidar_topic", "/points");

  const double summary_period_ms = this->declare_parameter<double>("summary_period_ms", 2000.0);
  const double safe_summary_period_ms = std::max(summary_period_ms, 100.0);

  const double imu_gap_threshold_ms = this->declare_parameter<double>("imu_gap_threshold_ms", 100.0);
  const double safe_imu_gap_threshold_ms = std::max(imu_gap_threshold_ms, 1.0);
  imu_timing_tracker_.SetGapThresholdMs(safe_imu_gap_threshold_ms);

  const double lidar_gap_threshold_ms = this->declare_parameter<double>("lidar_gap_threshold_ms", 500.0);
  const double safe_lidar_gap_threshold_ms = std::max(lidar_gap_threshold_ms, 1.0);
  lidar_timing_tracker_.SetGapThresholdMs(safe_lidar_gap_threshold_ms);

  const double lidar_scan_duration_ms = this->declare_parameter<double>("lidar_scan_duration_ms", 100.0);
  const double safe_lidar_scan_duration_ms = std::max(lidar_scan_duration_ms, 1.0);

  const double lidar_min_measured_scan_duration_ms = this->declare_parameter<double>("lidar_min_measured_scan_duration_ms", 1.0);
  const double safe_lidar_min_measured_scan_duration_ms = std::max(lidar_min_measured_scan_duration_ms, 0.1);

  const double lidar_max_measured_scan_duration_ms = this->declare_parameter<double>("lidar_max_measured_scan_duration_ms", 500.0);
  const double safe_lidar_max_measured_scan_duration_ms =
      std::max(lidar_max_measured_scan_duration_ms, safe_lidar_min_measured_scan_duration_ms);

  const bool lidar_prefer_measured_header_period = this->declare_parameter<bool>("lidar_prefer_measured_header_period", true);

  const std::string lidar_stamp_policy = this->declare_parameter<std::string>("lidar_stamp_policy", "scan_end");
  const auto parsed_lidar_stamp_policy = ParseLidarStampPolicy(lidar_stamp_policy);

  lidar_scan_window_estimator_.SetConfig(lidar::LidarScanWindowEstimatorConfig{
      .fallback_scan_duration_ms = safe_lidar_scan_duration_ms,
      .min_measured_scan_duration_ms = safe_lidar_min_measured_scan_duration_ms,
      .max_measured_scan_duration_ms = safe_lidar_max_measured_scan_duration_ms,
      .stamp_policy = parsed_lidar_stamp_policy,
      .prefer_measured_header_period = lidar_prefer_measured_header_period,
  });

  const double imu_buffer_retention_ms = this->declare_parameter<double>("imu_buffer_retention_ms", 5000.0);
  const double safe_imu_buffer_retention_ms = std::max(imu_buffer_retention_ms, 100.0);
  imu_sample_buffer_ = coverage::ImuSampleBuffer{MillisecondsToNanoseconds(safe_imu_buffer_retention_ms)};

  const double expected_imu_period_ms = this->declare_parameter<double>("expected_imu_period_ms", 5.0);
  const double safe_expected_imu_period_ms = std::max(expected_imu_period_ms, 0.1);

  const double max_missing_prefix_ms = this->declare_parameter<double>("max_missing_prefix_ms", 2.0 * safe_expected_imu_period_ms);
  const double max_missing_suffix_ms = this->declare_parameter<double>("max_missing_suffix_ms", 2.0 * safe_expected_imu_period_ms);
  const double max_internal_gap_ms = this->declare_parameter<double>("max_internal_gap_ms", 5.0 * safe_expected_imu_period_ms);

  imu_coverage_analyzer_.SetConfig(coverage::ImuCoverageConfig{
      .max_missing_prefix_ms = std::max(max_missing_prefix_ms, 0.0),
      .max_missing_suffix_ms = std::max(max_missing_suffix_ms, 0.0),
      .max_internal_gap_ms = std::max(max_internal_gap_ms, safe_expected_imu_period_ms),
  });

  imu_subscription_ =
      this->create_subscription<ImuMsg>(imu_topic, rclcpp::SensorDataQoS{}, [this](ImuMsg::ConstSharedPtr msg) { OnImuReceived(msg); });

  lidar_subscription_ = this->create_subscription<PointCloud2Msg>(lidar_topic, rclcpp::SensorDataQoS{},
                                                                  [this](PointCloud2Msg::ConstSharedPtr msg) { OnLidarReceived(msg); });

  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_summary_period_ms)),
      [this]() { OnTimer(); });

  RCLCPP_INFO(this->get_logger(),
              "TemporalMonitorNode started"
              " | imu_topic=%s"
              " | lidar_topic=%s"
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
              imu_topic.c_str(), lidar_topic.c_str(), safe_summary_period_ms, safe_imu_gap_threshold_ms, safe_lidar_gap_threshold_ms,
              safe_lidar_scan_duration_ms, safe_lidar_min_measured_scan_duration_ms, safe_lidar_max_measured_scan_duration_ms,
              lidar_prefer_measured_header_period ? "true" : "false", lidar::ToString(parsed_lidar_stamp_policy),
              safe_expected_imu_period_ms, safe_imu_buffer_retention_ms, std::max(max_missing_prefix_ms, 0.0),
              std::max(max_missing_suffix_ms, 0.0), std::max(max_internal_gap_ms, safe_expected_imu_period_ms));
}

void TemporalMonitorNode::OnImuReceived(ImuMsg::ConstSharedPtr msg) {
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

  imu_timing_tracker_.Observe(telemetry::TimingSample{
      .header_stamp_ns = stamp_ns,
      .receive_time_ns = this->now().nanoseconds(),
  });

  imu_sample_buffer_.Add(coverage::ImuSample{
      .stamp_ns = stamp_ns,
  });
}

void TemporalMonitorNode::OnLidarReceived(PointCloud2Msg::ConstSharedPtr msg) {
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

  if (!has_logged_lidar_point_cloud2_fields_) {
    const auto inspection = point_cloud2_field_inspector_.Inspect(*msg);
    LogPointCloud2FieldInspection(this->get_logger(), inspection);

    latest_lidar_point_time_diagnostics_ = BuildPointTimeDiagnostics(inspection);

    if (inspection.primary_time_field.has_value()) {
      lidar_point_time_field_ = *inspection.primary_time_field;
    }

    has_logged_lidar_point_cloud2_fields_ = true;
  }

  lidar_timing_tracker_.Observe(telemetry::TimingSample{
      .header_stamp_ns = stamp_ns,
      .receive_time_ns = this->now().nanoseconds(),
  });

  latest_lidar_scan_window_estimate_ = lidar_scan_window_estimator_.Estimate(stamp_ns);

  if (lidar_point_time_field_.has_value()) {
    const auto extraction = point_cloud2_time_field_extractor_.Extract(*msg, *lidar_point_time_field_);

    latest_lidar_point_time_diagnostics_.extraction_attempted = true;
    latest_lidar_point_time_diagnostics_.extraction_used = extraction.has_scan_window;
    latest_lidar_point_time_diagnostics_.extraction_reason = extraction.reason;
    latest_lidar_point_time_diagnostics_.extraction_unit = ros_adapters::ToString(extraction.time_unit);

    if (extraction.has_scan_window) {
      latest_lidar_scan_window_estimate_ = BuildPointTimeFieldEstimate(extraction);
    }
  }

  latest_imu_coverage_summary_ = imu_coverage_analyzer_.Analyze(latest_lidar_scan_window_estimate_.window, imu_sample_buffer_.Samples());

  has_lidar_coverage_summary_ = true;
}

void TemporalMonitorNode::OnTimer() {
  const auto imu_summary = imu_timing_tracker_.ConsumeWindowSummary();
  const auto lidar_summary = lidar_timing_tracker_.ConsumeWindowSummary();

  LogTimingSummary(this->get_logger(), "IMU", imu_summary);
  LogTimingSummary(this->get_logger(), "LiDAR", lidar_summary);
  LogImuCoverageSummary(this->get_logger(), has_lidar_coverage_summary_, latest_imu_coverage_summary_, latest_lidar_scan_window_estimate_,
                        imu_sample_buffer_.Size());

  const diagnostics::TemporalDiagnosticsBuilder diagnostics_builder;
  const auto diagnostic_snapshot = diagnostics_builder.Build(diagnostics::TemporalDiagnosticsInput{
      .imu_timing = imu_summary,
      .lidar_timing = lidar_summary,
      .has_imu_coverage = has_lidar_coverage_summary_,
      .imu_coverage = latest_imu_coverage_summary_,
      .lidar_scan_window = latest_lidar_scan_window_estimate_,
      .lidar_point_time = latest_lidar_point_time_diagnostics_,
      .imu_buffer_size = imu_sample_buffer_.Size(),
  });

  const render::ConsoleTemporalSummaryRenderer renderer;
  RCLCPP_INFO_STREAM(this->get_logger(), "\n" << renderer.Render(diagnostic_snapshot));
}

}  // namespace causal_slam::nodes