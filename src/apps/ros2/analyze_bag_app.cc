#include "apps/ros2/analyze_bag_app.h"

#include "application/offline_analysis/offline_stream_timing_fault_analyzer.h"

#include "apps/ros2/analyze_bag_cli.h"
#include "apps/ros2/bag_topic_inspector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "adapters/ros2/point_cloud2_conversions.h"
#include "domain/sensors/imu/imu_coverage_analyzer.h"
#include "domain/sensors/lidar/lidar_scan_window_estimator.h"
#include "domain/time/time_units.h"
#include "presentation/render/offline_temporal_report_artifact_writer.h"
#include "presentation/render/offline_temporal_report_console_renderer.h"

namespace {

using causal_slam::apps::ros2::AnalyzeBagOptions;
using causal_slam::offline_analysis::BagSummary;
using causal_slam::offline_analysis::BuildStreamTimingFaultReport;
using causal_slam::offline_analysis::DatasetVerdict;
using causal_slam::offline_analysis::LidarFirstCloudSummary;
using causal_slam::offline_analysis::LidarScanWindowSummary;
using causal_slam::offline_analysis::OfflineImuCoverageReport;
using causal_slam::offline_analysis::PointCloudCapabilitySummary;
using causal_slam::offline_analysis::PointFieldSummary;
using causal_slam::offline_analysis::TimingSummary;
using causal_slam::offline_analysis::TopicSummary;

std::int64_t ToNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return (static_cast<std::int64_t>(stamp.sec) * 1000000000LL) + static_cast<std::int64_t>(stamp.nanosec);
}

double NsToMs(std::int64_t value_ns) {
  return static_cast<double>(value_ns) / 1.0e6;
}

std::string PointFieldDatatypeName(std::uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return "INT8";
    case sensor_msgs::msg::PointField::UINT8:
      return "UINT8";
    case sensor_msgs::msg::PointField::INT16:
      return "INT16";
    case sensor_msgs::msg::PointField::UINT16:
      return "UINT16";
    case sensor_msgs::msg::PointField::INT32:
      return "INT32";
    case sensor_msgs::msg::PointField::UINT32:
      return "UINT32";
    case sensor_msgs::msg::PointField::FLOAT32:
      return "FLOAT32";
    case sensor_msgs::msg::PointField::FLOAT64:
      return "FLOAT64";
    default:
      return "UNKNOWN";
  }
}

causal_slam::pointcloud::PointCloud2TimeFieldRole TimeRoleForFieldName(const std::string& name) {
  if (name == "timestamp") {
    return causal_slam::pointcloud::PointCloud2TimeFieldRole::kPointTime;
  }

  if (name == "t" || name == "offset_time" || name == "time") {
    return causal_slam::pointcloud::PointCloud2TimeFieldRole::kPointOffsetTime;
  }

  return causal_slam::pointcloud::PointCloud2TimeFieldRole::kNone;
}

bool IsSupportedPointTimeField(const PointFieldSummary& field) {
  const auto role = TimeRoleForFieldName(field.name);

  if (role == causal_slam::pointcloud::PointCloud2TimeFieldRole::kPointTime) {
    return field.datatype == sensor_msgs::msg::PointField::FLOAT64;
  }

  if (role == causal_slam::pointcloud::PointCloud2TimeFieldRole::kPointOffsetTime) {
    return field.datatype == sensor_msgs::msg::PointField::UINT32 || field.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
           field.datatype == sensor_msgs::msg::PointField::FLOAT64;
  }

  return false;
}

PointCloudCapabilitySummary AnalyzePointCloudCapabilities(const std::vector<PointFieldSummary>& fields) {
  PointCloudCapabilitySummary result;

  for (const auto& field : fields) {
    if (field.name == "x") {
      result.has_x = true;
    } else if (field.name == "y") {
      result.has_y = true;
    } else if (field.name == "z") {
      result.has_z = true;
    } else if (field.name == "intensity") {
      result.has_intensity = true;
    } else if (field.name == "ring") {
      result.has_ring = true;
    } else if (field.name == "line") {
      result.has_line = true;
    } else if (field.name == "channel") {
      result.has_channel = true;
    }

    const auto role = TimeRoleForFieldName(field.name);
    if (role == causal_slam::pointcloud::PointCloud2TimeFieldRole::kNone) {
      continue;
    }

    if (!result.has_time_field) {
      result.has_time_field = true;
      result.time_field_name = field.name;
      result.time_field_datatype = field.datatype_name;
      result.point_time_role =
          role == causal_slam::pointcloud::PointCloud2TimeFieldRole::kPointTime ? "absolute_point_time" : "relative_point_offset_time";
      result.point_time_supported = IsSupportedPointTimeField(field);
    }
  }

  if (!result.has_time_field) {
    result.reason = "no point time field";
  } else if (result.point_time_supported && result.point_time_role == "absolute_point_time") {
    result.reason = "absolute point timestamp field is supported";
  } else if (result.point_time_supported) {
    result.reason = "relative point time field is supported";
  } else {
    result.reason = "point time field datatype is not supported";
  }

  return result;
}

class TimingAccumulator {
 public:
  void Observe(std::int64_t stamp_ns) {
    ++summary_.deserialized_messages;

    if (!summary_.has_first_stamp) {
      summary_.has_first_stamp = true;
      summary_.has_last_stamp = true;
      summary_.first_header_stamp_ns = stamp_ns;
      summary_.last_header_stamp_ns = stamp_ns;
      previous_stamp_ns_ = stamp_ns;
      return;
    }

    if (stamp_ns < previous_stamp_ns_) {
      ++summary_.reorder_count;
    }

    const auto period_ns = stamp_ns - previous_stamp_ns_;
    if (period_ns >= 0) {
      periods_ms_.push_back(NsToMs(period_ns));
    }

    previous_stamp_ns_ = stamp_ns;
    summary_.last_header_stamp_ns = stamp_ns;
  }

  void ObserveFailure() { ++summary_.deserialization_failures; }

  TimingSummary Finish() {
    if (periods_ms_.empty()) {
      return summary_;
    }

    summary_.has_period = true;
    summary_.period_count = periods_ms_.size();

    const auto [min_it, max_it] = std::minmax_element(periods_ms_.begin(), periods_ms_.end());

    long double sum = 0.0;
    for (double value : periods_ms_) {
      sum += value;
    }
    const long double mean = sum / static_cast<long double>(periods_ms_.size());

    long double squared_error_sum = 0.0;
    for (double value : periods_ms_) {
      const long double error = static_cast<long double>(value) - mean;
      squared_error_sum += error * error;
    }

    const long double variance = squared_error_sum / static_cast<long double>(periods_ms_.size());

    summary_.period_mean_ms = static_cast<double>(mean);
    summary_.period_min_ms = *min_it;
    summary_.period_max_ms = *max_it;
    summary_.period_stddev_ms = static_cast<double>(std::sqrt(variance));
    summary_.jitter_stddev_ms = summary_.period_stddev_ms;

    return summary_;
  }

 private:
  TimingSummary summary_;
  std::int64_t previous_stamp_ns_ = 0;
  std::vector<double> periods_ms_;
};

class LidarScanWindowAccumulator {
 public:
  void ObservePointTimeExtraction(const causal_slam::pointcloud::PointCloud2TimeFieldExtraction& extraction) {
    ++summary_.scans_total;
    summary_.last_reason = extraction.reason;
    summary_.time_unit = causal_slam::pointcloud::ToString(extraction.time_unit);

    if (!extraction.has_scan_window) {
      ++summary_.failed;
      return;
    }

    ++summary_.estimated;
    ++summary_.with_point_time;
    summary_.source = "point_time_field";
    summary_.confidence = "HIGH";
    summary_.points_total += extraction.point_count_total;
    summary_.points_used += extraction.point_count_used;

    const double duration_ms =
        causal_slam::core::NanosecondsToMilliseconds(extraction.scan_window.end_ns - extraction.scan_window.start_ns);

    durations_ms_.push_back(duration_ms);
    ObservePointTimeDurationOutlier(summary_.scans_total - 1, duration_ms);
  }

  void ObserveFallbackEstimate(const causal_slam::lidar::LidarScanWindowEstimate& estimate) {
    ++summary_.scans_total;
    summary_.source = causal_slam::lidar::ToString(estimate.source);
    summary_.confidence = causal_slam::lidar::ToString(estimate.confidence);
    summary_.time_unit = "none";
    summary_.last_reason = estimate.reason;

    if (!estimate.window.IsValid() || estimate.duration_ms <= 0.0) {
      ++summary_.failed;
      return;
    }

    ++summary_.estimated;
    durations_ms_.push_back(estimate.duration_ms);
  }

  LidarScanWindowSummary Finish() {
    if (durations_ms_.empty()) {
      return summary_;
    }

    summary_.has_duration = true;

    const auto [min_it, max_it] = std::minmax_element(durations_ms_.begin(), durations_ms_.end());

    long double sum = 0.0;
    for (double value : durations_ms_) {
      sum += value;
    }
    const long double mean = sum / static_cast<long double>(durations_ms_.size());

    long double squared_error_sum = 0.0;
    for (double value : durations_ms_) {
      const long double error = static_cast<long double>(value) - mean;
      squared_error_sum += error * error;
    }

    const long double variance = squared_error_sum / static_cast<long double>(durations_ms_.size());

    summary_.duration_mean_ms = static_cast<double>(mean);
    summary_.duration_min_ms = *min_it;
    summary_.duration_max_ms = *max_it;
    summary_.duration_stddev_ms = static_cast<double>(std::sqrt(variance));
    if (summary_.scans_total > 0) {
      summary_.duration_outlier_ratio = static_cast<double>(summary_.duration_outlier_count) / static_cast<double>(summary_.scans_total);
    }
    return summary_;
  }

 private:
  void ObservePointTimeDurationOutlier(std::uint64_t scan_index, double duration_ms) {
    if (duration_ms <= summary_.duration_outlier_threshold_ms) {
      return;
    }

    ++summary_.duration_outlier_count;

    if (duration_ms > summary_.worst_duration_outlier_ms) {
      summary_.worst_duration_outlier_ms = duration_ms;
      summary_.worst_duration_outlier_scan_index = scan_index;
    }
  }
  LidarScanWindowSummary summary_;
  std::vector<double> durations_ms_;
};

std::vector<PointFieldSummary> SummarizeFields(const sensor_msgs::msg::PointCloud2& cloud) {
  std::vector<PointFieldSummary> fields;
  fields.reserve(cloud.fields.size());

  for (const auto& field : cloud.fields) {
    fields.push_back(PointFieldSummary{
        .name = field.name,
        .offset = field.offset,
        .datatype = field.datatype,
        .count = field.count,
        .datatype_name = PointFieldDatatypeName(field.datatype),
    });
  }

  return fields;
}

struct ImuBoundsInWindow {
  bool found = false;
  std::int64_t first_ns = 0;
  std::int64_t last_ns = 0;
};

ImuBoundsInWindow FindImuBoundsInWindow(causal_slam::core::TimeWindow scan_window,
                                        const std::vector<causal_slam::coverage::ImuSample>& imu_samples) {
  ImuBoundsInWindow bounds;

  for (const auto& sample : imu_samples) {
    if (!scan_window.Contains(sample.stamp_ns)) {
      continue;
    }

    if (!bounds.found) {
      bounds.found = true;
      bounds.first_ns = sample.stamp_ns;
      bounds.last_ns = sample.stamp_ns;
      continue;
    }

    bounds.first_ns = std::min(bounds.first_ns, sample.stamp_ns);
    bounds.last_ns = std::max(bounds.last_ns, sample.stamp_ns);
  }

  return bounds;
}

OfflineImuCoverageReport BuildOfflineImuCoverageReport(const std::vector<causal_slam::core::TimeWindow>& scan_windows,
                                                       const std::vector<causal_slam::coverage::ImuSample>& imu_samples) {
  OfflineImuCoverageReport report;
  report.scans_total = scan_windows.size();

  if (scan_windows.empty()) {
    report.worst_reason = "no_scan_windows";
    return report;
  }

  const auto coverage_config = causal_slam::coverage::ImuCoverageConfig{
      .max_missing_prefix_ms = 12.5,
      .max_missing_suffix_ms = 12.5,
      .max_internal_gap_ms = 30.0,
  };
  causal_slam::coverage::ImuCoverageAnalyzer analyzer{coverage_config};

  std::uint64_t imu_count_sum = 0;
  double coverage_ratio_sum = 0.0;
  bool initialized_extremes = false;

  for (std::size_t scan_index = 0; scan_index < scan_windows.size(); ++scan_index) {
    const auto& scan_window = scan_windows[scan_index];
    const auto coverage = analyzer.Analyze(scan_window, imu_samples);

    imu_count_sum += coverage.imu_count_in_window;
    coverage_ratio_sum += coverage.coverage_ratio;

    if (!initialized_extremes) {
      initialized_extremes = true;
      report.min_imu_count_in_window = coverage.imu_count_in_window;
      report.max_imu_count_in_window = coverage.imu_count_in_window;
      report.min_coverage_ratio = coverage.coverage_ratio;
    } else {
      report.min_imu_count_in_window = std::min(report.min_imu_count_in_window, coverage.imu_count_in_window);
      report.max_imu_count_in_window = std::max(report.max_imu_count_in_window, coverage.imu_count_in_window);
      report.min_coverage_ratio = std::min(report.min_coverage_ratio, coverage.coverage_ratio);
    }

    if (coverage.missing_prefix_ms > coverage_config.max_missing_prefix_ms) {
      ++report.missing_prefix_count;
    }
    if (coverage.missing_suffix_ms > coverage_config.max_missing_suffix_ms) {
      ++report.missing_suffix_count;
    }
    if (coverage.max_gap_inside_ms > coverage_config.max_internal_gap_ms) {
      ++report.internal_gap_count;
    }

    if (coverage.health != causal_slam::coverage::ImuCoverageHealth::kOk) {
      if (!report.has_worst_sample || coverage.coverage_ratio < report.worst_coverage_ratio) {
        const auto imu_bounds = FindImuBoundsInWindow(scan_window, imu_samples);

        report.has_worst_sample = true;
        report.worst_scan_index = static_cast<std::uint64_t>(scan_index);
        report.worst_sample_reason = coverage.reason;
        report.worst_scan_start_ns = scan_window.start_ns;
        report.worst_scan_end_ns = scan_window.end_ns;
        report.worst_has_imu_bounds = imu_bounds.found;
        report.worst_first_imu_in_window_ns = imu_bounds.first_ns;
        report.worst_last_imu_in_window_ns = imu_bounds.last_ns;
        report.worst_imu_count_in_window = coverage.imu_count_in_window;
        report.worst_missing_prefix_ms = coverage.missing_prefix_ms;
        report.worst_missing_suffix_ms = coverage.missing_suffix_ms;
        report.worst_max_gap_inside_ms = coverage.max_gap_inside_ms;
        report.worst_coverage_ratio = coverage.coverage_ratio;
      }
    }

    switch (coverage.health) {
      case causal_slam::coverage::ImuCoverageHealth::kOk:
        ++report.ok;
        break;
      case causal_slam::coverage::ImuCoverageHealth::kWarning:
        ++report.warning;
        ++report.fault_reasons[coverage.reason];
        break;
      case causal_slam::coverage::ImuCoverageHealth::kDegraded:
        if (coverage.reason == "invalid_scan_window") {
          ++report.invalid;
        } else {
          ++report.degraded;
        }
        ++report.fault_reasons[coverage.reason];
        if (report.worst_reason == "ok" || report.worst_reason == "not_analyzed") {
          report.worst_reason = coverage.reason;
        }
        break;
    }
  }

  report.mean_imu_count_in_window = static_cast<double>(imu_count_sum) / static_cast<double>(scan_windows.size());
  report.mean_coverage_ratio = coverage_ratio_sum / static_cast<double>(scan_windows.size());

  if (report.warning == 0 && report.degraded == 0 && report.invalid == 0) {
    report.worst_reason = "ok";
  } else if (report.worst_reason == "not_analyzed") {
    report.worst_reason = report.worst_sample_reason;
  }

  return report;
}

DatasetVerdict BuildDatasetVerdict(const BagSummary& summary) {
  DatasetVerdict verdict;

  if (!summary.lidar_topic_found || !summary.imu_topic_found) {
    verdict.health = "INVALID";
    verdict.reason = "selected_topic_missing";
    verdict.map_update_recommended = false;
    return verdict;
  }

  if (summary.lidar_scan_windows.scans_total == 0 || summary.lidar_scan_windows.estimated == 0 || summary.imu_coverage.scans_total == 0) {
    verdict.health = "INVALID";
    verdict.reason = "no_scan_windows";
    verdict.map_update_recommended = false;
    return verdict;
  }

  const std::uint64_t fault_count = summary.imu_coverage.warning + summary.imu_coverage.degraded + summary.imu_coverage.invalid;

  verdict.fault_ratio = static_cast<double>(fault_count) / static_cast<double>(summary.imu_coverage.scans_total);
  const double lidar_duration_outlier_ratio = summary.lidar_scan_windows.scans_total == 0
                                                  ? 0.0
                                                  : static_cast<double>(summary.lidar_scan_windows.duration_outlier_count) /
                                                        static_cast<double>(summary.lidar_scan_windows.scans_total);

  const bool has_stream_timing_faults = !summary.stream_timing_faults.fault_reasons.empty();

  if (lidar_duration_outlier_ratio >= 0.05) {
    verdict.health = "DEGRADED";
    verdict.reason = "lidar_scan_window_duration_outliers";
    verdict.map_update_recommended = false;
    return verdict;
  }

  if (summary.lidar_scan_windows.duration_outlier_count > 0) {
    verdict.health = "WARNING";
    verdict.reason = "lidar_scan_window_duration_outliers";
    verdict.map_update_recommended = true;
    return verdict;
  }
  if (summary.imu_coverage.invalid > 0) {
    verdict.health = "INVALID";
    verdict.reason = "invalid_imu_coverage_windows";
    verdict.map_update_recommended = false;
    return verdict;
  }

  if (verdict.fault_ratio >= 0.05) {
    verdict.health = "DEGRADED";
    verdict.reason = "imu_coverage_degraded";
    verdict.map_update_recommended = false;
    return verdict;
  }

  if (fault_count > 0) {
    verdict.health = "WARNING";
    verdict.reason =
        has_stream_timing_faults ? "lidar_stream_timing_warning_with_isolated_imu_coverage_faults" : "isolated_imu_coverage_faults";
    verdict.map_update_recommended = true;
    return verdict;
  }

  if (has_stream_timing_faults) {
    verdict.health = "WARNING";
    verdict.reason = "lidar_stream_timing_warning";
    verdict.map_update_recommended = true;
    return verdict;
  }

  if (summary.lidar_scan_windows.with_point_time == 0) {
    verdict.health = "WARNING";
    verdict.reason = "fallback_scan_window_used";
    verdict.map_update_recommended = true;
    return verdict;
  }

  verdict.health = "OK";
  verdict.reason = "ok";
  verdict.map_update_recommended = true;
  return verdict;
}

BagSummary BuildSummary(const std::vector<rosbag2_storage::TopicMetadata>& metadata, const std::map<std::string, std::uint64_t>& counts,
                        const AnalyzeBagOptions& options) {
  BagSummary summary;

  for (const auto& topic : metadata) {
    TopicSummary topic_summary;
    topic_summary.name = topic.name;
    topic_summary.type = topic.type;

    if (const auto it = counts.find(topic.name); it != counts.end()) {
      topic_summary.message_count = it->second;
    }

    if (topic.name == options.lidar_topic) {
      summary.lidar_topic_found = true;
      summary.lidar_messages = topic_summary.message_count;
    }

    if (topic.name == options.imu_topic) {
      summary.imu_topic_found = true;
      summary.imu_messages = topic_summary.message_count;
    }

    summary.topics.push_back(std::move(topic_summary));
  }

  return summary;
}

int AnalyzeBag(const AnalyzeBagOptions& options, std::ostream& out, std::ostream& err) {
  try {
    rosbag2_cpp::Reader reader;
    reader.open(options.bag_path);

    const auto metadata = reader.get_all_topics_and_types();

    std::map<std::string, std::uint64_t> counts;
    TimingAccumulator imu_timing;
    TimingAccumulator lidar_timing;
    LidarScanWindowAccumulator lidar_scan_windows;

    causal_slam::lidar::LidarScanWindowEstimator fallback_scan_window_estimator{causal_slam::lidar::LidarScanWindowEstimatorConfig{
        .fallback_scan_duration_ms = 100.0,
        .min_measured_scan_duration_ms = 1.0,
        .max_measured_scan_duration_ms = 500.0,
        .stamp_policy = causal_slam::lidar::LidarStampPolicy::kScanEnd,
        .prefer_measured_header_period = true,
    }};

    causal_slam::pointcloud::PointCloud2TimeFieldExtractor time_field_extractor;

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> lidar_serializer;

    LidarFirstCloudSummary first_cloud;
    std::vector<causal_slam::coverage::ImuSample> imu_samples;
    std::vector<causal_slam::core::TimeWindow> scan_windows_for_coverage;

    while (reader.has_next()) {
      const auto message = reader.read_next();
      ++counts[message->topic_name];

      if (message->topic_name == options.imu_topic) {
        try {
          rclcpp::SerializedMessage serialized_message{*message->serialized_data};
          sensor_msgs::msg::Imu imu_message;
          imu_serializer.deserialize_message(&serialized_message, &imu_message);

          const auto stamp_ns = ToNanoseconds(imu_message.header.stamp);
          imu_timing.Observe(stamp_ns);
          imu_samples.push_back(causal_slam::coverage::ImuSample{.stamp_ns = stamp_ns});
        } catch (const std::exception&) {
          imu_timing.ObserveFailure();
        }
      }

      if (message->topic_name == options.lidar_topic) {
        try {
          rclcpp::SerializedMessage serialized_message{*message->serialized_data};
          sensor_msgs::msg::PointCloud2 cloud_message;
          lidar_serializer.deserialize_message(&serialized_message, &cloud_message);

          const auto stamp_ns = ToNanoseconds(cloud_message.header.stamp);
          lidar_timing.Observe(stamp_ns);

          if (!first_cloud.observed) {
            first_cloud.observed = true;
            first_cloud.frame_id = cloud_message.header.frame_id;
            first_cloud.width = cloud_message.width;
            first_cloud.height = cloud_message.height;
            first_cloud.point_step = cloud_message.point_step;
            first_cloud.row_step = cloud_message.row_step;
            first_cloud.data_size = cloud_message.data.size();
            first_cloud.fields_count = cloud_message.fields.size();
            first_cloud.fields = SummarizeFields(cloud_message);
            first_cloud.capabilities = AnalyzePointCloudCapabilities(first_cloud.fields);
          }

          const auto cloud_view = causal_slam::ros_adapters::ToPointCloud2CloudView(cloud_message);
          const auto fields = causal_slam::ros_adapters::ToPointCloud2FieldInfos(cloud_message);

          bool used_point_time_field = false;

          for (auto field : fields) {
            field.time_role = TimeRoleForFieldName(field.name);
            if (field.time_role == causal_slam::pointcloud::PointCloud2TimeFieldRole::kNone) {
              continue;
            }

            used_point_time_field = true;
            const auto extraction = time_field_extractor.Extract(cloud_view, field);
            lidar_scan_windows.ObservePointTimeExtraction(extraction);
            if (extraction.has_scan_window) {
              scan_windows_for_coverage.push_back(extraction.scan_window);
            }
            break;
          }

          if (!used_point_time_field) {
            const auto estimate = fallback_scan_window_estimator.Estimate(stamp_ns);
            lidar_scan_windows.ObserveFallbackEstimate(estimate);
            if (estimate.window.IsValid()) {
              scan_windows_for_coverage.push_back(estimate.window);
            }
          }
        } catch (const std::exception&) {
          lidar_timing.ObserveFailure();
        }
      }
    }

    BagSummary summary = BuildSummary(metadata, counts, options);
    summary.imu_timing = imu_timing.Finish();
    summary.lidar_timing = lidar_timing.Finish();
    summary.lidar_first_cloud = std::move(first_cloud);
    summary.lidar_scan_windows = lidar_scan_windows.Finish();
    summary.imu_coverage = BuildOfflineImuCoverageReport(scan_windows_for_coverage, imu_samples);
    summary.stream_timing_faults = BuildStreamTimingFaultReport(summary);
    summary.verdict = BuildDatasetVerdict(summary);

    const causal_slam::render::OfflineTemporalReportArtifactWriter artifact_writer;
    const causal_slam::render::OfflineTemporalReportArtifactPaths artifact_paths{
        .json_report_path = options.report_path,
        .html_report_path = options.html_report_path,
    };
    const causal_slam::render::OfflineTemporalReportArtifactContext artifact_context{
        .bag_path = options.bag_path,
        .lidar_topic = options.lidar_topic,
        .imu_topic = options.imu_topic,
    };
    if (!artifact_writer.Write(artifact_paths, artifact_context, summary, err)) {
      return 3;
    }

    const causal_slam::render::OfflineTemporalReportConsoleRenderer console_renderer;
    const causal_slam::render::OfflineTemporalReportConsoleRenderContext console_context{
        .bag_path = options.bag_path,
        .report_path = options.report_path,
        .lidar_topic = options.lidar_topic,
        .imu_topic = options.imu_topic,
    };
    out << console_renderer.Render(console_context, summary);
    return 0;
  } catch (const std::exception& e) {
    err << "Failed to analyze rosbag2.\n"
        << "  bag: " << options.bag_path << "\n"
        << "  error: " << e.what() << "\n";
    return 2;
  }
}

int ListTopics(const AnalyzeBagOptions& options, std::ostream& out, std::ostream& err) {
  try {
    const auto inspection = causal_slam::apps::ros2::InspectBagTopics(options.bag_path);

    out << "Bag topics:\n";

    if (inspection.topics.empty()) {
      out << "  none\n";
      return 0;
    }

    for (const auto& topic : inspection.topics) {
      out << "  " << topic.name << " " << topic.type << " messages=" << topic.message_count << "\n";
    }

    return 0;
  } catch (const std::exception& e) {
    err << "Failed to list rosbag2 topics.\n"
        << "  bag: " << options.bag_path << "\n"
        << "  error: " << e.what() << "\n";
    return 2;
  }
}

}  // namespace

int causal_slam::apps::ros2::RunAnalyzeBagCli(int argc, char** argv, std::ostream& out, std::ostream& err) {
  const auto options = ParseAnalyzeBagArgs(argc, argv, err);
  if (!options) {
    return 2;
  }

  if (options->show_help) {
    PrintUsage(out);
    return 0;
  }

  if (options->list_topics) {
    return ListTopics(*options, out, err);
  }

  return AnalyzeBag(*options, out, err);
}