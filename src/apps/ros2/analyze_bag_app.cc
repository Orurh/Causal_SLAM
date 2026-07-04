#include "apps/ros2/analyze_bag_app.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
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

namespace {

struct AnalyzeBagOptions {
  bool show_help = false;
  std::string bag_path;
  std::string lidar_topic;
  std::string imu_topic;
  std::string report_path;
};

struct TopicSummary {
  std::string name;
  std::string type;
  std::uint64_t message_count = 0;
};

struct TimingSummary {
  std::uint64_t deserialized_messages = 0;
  std::uint64_t deserialization_failures = 0;
  bool has_first_stamp = false;
  bool has_last_stamp = false;
  std::int64_t first_header_stamp_ns = 0;
  std::int64_t last_header_stamp_ns = 0;
  bool has_period = false;
  std::uint64_t period_count = 0;
  double period_mean_ms = 0.0;
  double period_min_ms = 0.0;
  double period_max_ms = 0.0;
  double period_stddev_ms = 0.0;
  double jitter_stddev_ms = 0.0;
  std::uint64_t reorder_count = 0;
};

struct PointFieldSummary {
  std::string name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
  std::string datatype_name;
};

struct PointCloudCapabilitySummary {
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  bool has_ring = false;
  bool has_line = false;
  bool has_channel = false;
  bool has_time_field = false;
  std::string time_field_name;
  std::string time_field_datatype;
  std::string point_time_role = "none";
  bool point_time_supported = false;
  std::string reason = "no point time field";
};

struct LidarFirstCloudSummary {
  bool observed = false;
  std::string frame_id;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  std::size_t data_size = 0;
  std::size_t fields_count = 0;
  std::vector<PointFieldSummary> fields;
  PointCloudCapabilitySummary capabilities;
};

struct LidarScanWindowSummary {
  std::uint64_t scans_total = 0;
  std::uint64_t estimated = 0;
  std::uint64_t with_point_time = 0;
  std::uint64_t failed = 0;
  std::uint64_t points_total = 0;
  std::uint64_t points_used = 0;
  bool has_duration = false;
  double duration_mean_ms = 0.0;
  double duration_min_ms = 0.0;
  double duration_max_ms = 0.0;
  double duration_stddev_ms = 0.0;
  std::string source = "none";
  std::string confidence = "UNKNOWN";
  std::string time_unit = "unknown";
  std::string last_reason = "not_started";
};

struct OfflineImuCoverageReport {
  std::uint64_t scans_total = 0;
  std::uint64_t ok = 0;
  std::uint64_t warning = 0;
  std::uint64_t degraded = 0;
  std::uint64_t invalid = 0;
  std::uint64_t missing_prefix_count = 0;
  std::uint64_t missing_suffix_count = 0;
  std::uint64_t internal_gap_count = 0;
  std::uint64_t min_imu_count_in_window = 0;
  std::uint64_t max_imu_count_in_window = 0;
  double mean_imu_count_in_window = 0.0;
  double min_coverage_ratio = 0.0;
  double mean_coverage_ratio = 0.0;
  std::string worst_reason = "not_analyzed";

  bool has_worst_sample = false;
  std::uint64_t worst_scan_index = 0;
  std::string worst_sample_reason = "none";
  std::int64_t worst_scan_start_ns = 0;
  std::int64_t worst_scan_end_ns = 0;
  bool worst_has_imu_bounds = false;
  std::int64_t worst_first_imu_in_window_ns = 0;
  std::int64_t worst_last_imu_in_window_ns = 0;
  std::uint64_t worst_imu_count_in_window = 0;
  double worst_missing_prefix_ms = 0.0;
  double worst_missing_suffix_ms = 0.0;
  double worst_max_gap_inside_ms = 0.0;
  double worst_coverage_ratio = 0.0;

  std::map<std::string, std::uint64_t> fault_reasons;
};

struct DatasetVerdict {
  std::string health = "UNKNOWN";
  std::string reason = "not_analyzed";
  double fault_ratio = 0.0;
  bool map_update_recommended = false;
};

struct BagSummary {
  std::vector<TopicSummary> topics;
  std::uint64_t lidar_messages = 0;
  std::uint64_t imu_messages = 0;
  bool lidar_topic_found = false;
  bool imu_topic_found = false;
  TimingSummary imu_timing;
  TimingSummary lidar_timing;
  LidarFirstCloudSummary lidar_first_cloud;
  LidarScanWindowSummary lidar_scan_windows;
  OfflineImuCoverageReport imu_coverage;
  DatasetVerdict verdict;
};

void PrintUsage(std::ostream& out) {
  out << "Usage:\n"
      << "  causal_slam_analyze_bag \\\n"
      << "    --bag <path> \\\n"
      << "    --lidar-topic <topic> \\\n"
      << "    --imu-topic <topic> \\\n"
      << "    --report <path>\n"
      << "\n"
      << "Options:\n"
      << "  --bag <path>           Path to rosbag2 directory\n"
      << "  --lidar-topic <topic>  LiDAR PointCloud2 topic\n"
      << "  --imu-topic <topic>    IMU topic\n"
      << "  --report <path>        Output JSON report path\n"
      << "  -h, --help             Show this help message\n";
}

bool IsHelpArg(std::string_view arg) {
  return arg == "--help" || arg == "-h";
}

bool IsOptionArg(std::string_view arg) {
  return arg == "--bag" || arg == "--lidar-topic" || arg == "--imu-topic" || arg == "--report";
}

std::optional<std::string> RequireValue(int argc, char** argv, int index, std::string_view option, std::ostream& err) {
  const int value_index = index + 1;
  if (value_index >= argc) {
    err << "Missing value for " << option << ".\n";
    return std::nullopt;
  }

  const std::string_view value{argv[value_index]};
  if (value.empty() || value.starts_with("--")) {
    err << "Missing value for " << option << ".\n";
    return std::nullopt;
  }

  return std::string{value};
}

std::optional<AnalyzeBagOptions> ParseArgs(int argc, char** argv, std::ostream& err) {
  AnalyzeBagOptions options;

  if (argc == 2 && IsHelpArg(argv[1])) {
    options.show_help = true;
    return options;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};

    if (IsHelpArg(arg)) {
      options.show_help = true;
      return options;
    }

    if (!IsOptionArg(arg)) {
      err << "Unknown argument: " << arg << ".\n";
      return std::nullopt;
    }

    const auto value = RequireValue(argc, argv, i, arg, err);
    if (!value.has_value()) {
      return std::nullopt;
    }

    if (arg == "--bag") {
      options.bag_path = *value;
    } else if (arg == "--lidar-topic") {
      options.lidar_topic = *value;
    } else if (arg == "--imu-topic") {
      options.imu_topic = *value;
    } else if (arg == "--report") {
      options.report_path = *value;
    }

    ++i;
  }

  bool valid = true;
  if (options.bag_path.empty()) {
    err << "Missing required argument: --bag <path>.\n";
    valid = false;
  }
  if (options.lidar_topic.empty()) {
    err << "Missing required argument: --lidar-topic <topic>.\n";
    valid = false;
  }
  if (options.imu_topic.empty()) {
    err << "Missing required argument: --imu-topic <topic>.\n";
    valid = false;
  }
  if (options.report_path.empty()) {
    err << "Missing required argument: --report <path>.\n";
    valid = false;
  }

  if (!valid) {
    return std::nullopt;
  }

  return options;
}

std::string JsonEscape(std::string_view value) {
  std::string escaped;
  escaped.reserve(value.size());

  for (const char c : value) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += c;
        break;
    }
  }

  return escaped;
}

void WriteJsonString(std::ostream& out, std::string_view value) {
  out << '"' << JsonEscape(value) << '"';
}

std::int64_t ToNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
}

double NsToMs(std::int64_t value_ns) {
  return static_cast<double>(value_ns) / 1.0e6;
}

void WriteNullableDouble(std::ostream& out, bool has_value, double value) {
  if (!has_value) {
    out << "null";
    return;
  }

  out << value;
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

    durations_ms_.push_back(causal_slam::core::NanosecondsToMilliseconds(extraction.scan_window.end_ns - extraction.scan_window.start_ns));
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

    return summary_;
  }

 private:
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
    verdict.reason = "isolated_imu_coverage_faults";
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

void WriteTimingJson(std::ostream& report, const TimingSummary& timing) {
  report << "    \"deserialized_messages\": " << timing.deserialized_messages << ",\n";
  report << "    \"deserialization_failures\": " << timing.deserialization_failures << ",\n";
  report << "    \"first_header_stamp_ns\": " << timing.first_header_stamp_ns << ",\n";
  report << "    \"last_header_stamp_ns\": " << timing.last_header_stamp_ns << ",\n";
  report << "    \"duration_ms\": "
         << (timing.has_first_stamp && timing.has_last_stamp ? NsToMs(timing.last_header_stamp_ns - timing.first_header_stamp_ns) : 0.0)
         << ",\n";
  report << "    \"period_count\": " << timing.period_count << ",\n";
  report << "    \"period_mean_ms\": " << timing.period_mean_ms << ",\n";
  report << "    \"period_min_ms\": " << timing.period_min_ms << ",\n";
  report << "    \"period_max_ms\": " << timing.period_max_ms << ",\n";
  report << "    \"period_stddev_ms\": " << timing.period_stddev_ms << ",\n";
  report << "    \"jitter_stddev_ms\": " << timing.jitter_stddev_ms << ",\n";
  report << "    \"reorder_count\": " << timing.reorder_count << "\n";
}

bool WriteReportJson(const AnalyzeBagOptions& options, const BagSummary& summary, std::ostream& err) {
  std::ofstream report{options.report_path};
  if (!report) {
    err << "Failed to open report file for writing: " << options.report_path << "\n";
    return false;
  }

  report << "{\n";
  report << "  \"tool\": \"causal_slam_analyze_bag\",\n";
  report << "  \"schema_version\": 1,\n";
  report << "  \"bag\": ";
  WriteJsonString(report, options.bag_path);
  report << ",\n";

  report << "  \"selected_topics\": {\n";
  report << "    \"lidar\": ";
  WriteJsonString(report, options.lidar_topic);
  report << ",\n";
  report << "    \"imu\": ";
  WriteJsonString(report, options.imu_topic);
  report << "\n";
  report << "  },\n";

  report << "  \"summary\": {\n";
  report << "    \"lidar_messages\": " << summary.lidar_messages << ",\n";
  report << "    \"imu_messages\": " << summary.imu_messages << ",\n";
  report << "    \"lidar_topic_found\": " << (summary.lidar_topic_found ? "true" : "false") << ",\n";
  report << "    \"imu_topic_found\": " << (summary.imu_topic_found ? "true" : "false") << "\n";
  report << "  },\n";

  report << "  \"verdict\": {\n";
  report << "    \"health\": ";
  WriteJsonString(report, summary.verdict.health);
  report << ",\n";
  report << "    \"reason\": ";
  WriteJsonString(report, summary.verdict.reason);
  report << ",\n";
  report << "    \"fault_ratio\": " << summary.verdict.fault_ratio << ",\n";
  report << "    \"map_update_recommended\": " << (summary.verdict.map_update_recommended ? "true" : "false") << "\n";
  report << "  },\n";

  report << "  \"imu_timing\": {\n";
  WriteTimingJson(report, summary.imu_timing);
  report << "  },\n";

  report << "  \"lidar_timing\": {\n";
  WriteTimingJson(report, summary.lidar_timing);
  report << "  },\n";

  report << "  \"lidar_first_cloud\": {\n";
  report << "    \"observed\": " << (summary.lidar_first_cloud.observed ? "true" : "false") << ",\n";
  report << "    \"frame_id\": ";
  WriteJsonString(report, summary.lidar_first_cloud.frame_id);
  report << ",\n";
  report << "    \"width\": " << summary.lidar_first_cloud.width << ",\n";
  report << "    \"height\": " << summary.lidar_first_cloud.height << ",\n";
  report << "    \"point_step\": " << summary.lidar_first_cloud.point_step << ",\n";
  report << "    \"row_step\": " << summary.lidar_first_cloud.row_step << ",\n";
  report << "    \"data_size\": " << summary.lidar_first_cloud.data_size << ",\n";
  report << "    \"fields_count\": " << summary.lidar_first_cloud.fields_count << "\n";
  report << "  },\n";

  const auto& caps = summary.lidar_first_cloud.capabilities;
  report << "  \"pointcloud_capabilities\": {\n";
  report << "    \"has_x\": " << (caps.has_x ? "true" : "false") << ",\n";
  report << "    \"has_y\": " << (caps.has_y ? "true" : "false") << ",\n";
  report << "    \"has_z\": " << (caps.has_z ? "true" : "false") << ",\n";
  report << "    \"has_intensity\": " << (caps.has_intensity ? "true" : "false") << ",\n";
  report << "    \"has_ring\": " << (caps.has_ring ? "true" : "false") << ",\n";
  report << "    \"has_line\": " << (caps.has_line ? "true" : "false") << ",\n";
  report << "    \"has_channel\": " << (caps.has_channel ? "true" : "false") << ",\n";
  report << "    \"has_time_field\": " << (caps.has_time_field ? "true" : "false") << ",\n";
  report << "    \"time_field_name\": ";
  WriteJsonString(report, caps.time_field_name);
  report << ",\n";
  report << "    \"time_field_datatype\": ";
  WriteJsonString(report, caps.time_field_datatype);
  report << ",\n";
  report << "    \"point_time_role\": ";
  WriteJsonString(report, caps.point_time_role);
  report << ",\n";
  report << "    \"point_time_supported\": " << (caps.point_time_supported ? "true" : "false") << ",\n";
  report << "    \"reason\": ";
  WriteJsonString(report, caps.reason);
  report << "\n";
  report << "  },\n";

  report << "  \"pointcloud_fields\": [\n";
  for (std::size_t i = 0; i < summary.lidar_first_cloud.fields.size(); ++i) {
    const auto& field = summary.lidar_first_cloud.fields[i];
    report << "    {\n";
    report << "      \"name\": ";
    WriteJsonString(report, field.name);
    report << ",\n";
    report << "      \"offset\": " << field.offset << ",\n";
    report << "      \"datatype\": " << static_cast<int>(field.datatype) << ",\n";
    report << "      \"datatype_name\": ";
    WriteJsonString(report, field.datatype_name);
    report << ",\n";
    report << "      \"count\": " << field.count << "\n";
    report << "    }";
    if (i + 1 < summary.lidar_first_cloud.fields.size()) {
      report << ",";
    }
    report << "\n";
  }
  report << "  ],\n";

  const auto& windows = summary.lidar_scan_windows;
  report << "  \"lidar_scan_windows\": {\n";
  report << "    \"scans_total\": " << windows.scans_total << ",\n";
  report << "    \"estimated\": " << windows.estimated << ",\n";
  report << "    \"with_point_time\": " << windows.with_point_time << ",\n";
  report << "    \"failed\": " << windows.failed << ",\n";
  report << "    \"points_total\": " << windows.points_total << ",\n";
  report << "    \"points_used\": " << windows.points_used << ",\n";
  report << "    \"duration_mean_ms\": ";
  WriteNullableDouble(report, windows.has_duration, windows.duration_mean_ms);
  report << ",\n";
  report << "    \"duration_min_ms\": ";
  WriteNullableDouble(report, windows.has_duration, windows.duration_min_ms);
  report << ",\n";
  report << "    \"duration_max_ms\": ";
  WriteNullableDouble(report, windows.has_duration, windows.duration_max_ms);
  report << ",\n";
  report << "    \"duration_stddev_ms\": ";
  WriteNullableDouble(report, windows.has_duration, windows.duration_stddev_ms);
  report << ",\n";
  report << "    \"source\": ";
  WriteJsonString(report, windows.source);
  report << ",\n";
  report << "    \"confidence\": ";
  WriteJsonString(report, windows.confidence);
  report << ",\n";
  report << "    \"time_unit\": ";
  WriteJsonString(report, windows.time_unit);
  report << ",\n";
  report << "    \"last_reason\": ";
  WriteJsonString(report, windows.last_reason);
  report << "\n";
  report << "  },\n";

  const auto& coverage = summary.imu_coverage;
  report << "  \"imu_coverage\": {\n";
  report << "    \"scans_total\": " << coverage.scans_total << ",\n";
  report << "    \"ok\": " << coverage.ok << ",\n";
  report << "    \"warning\": " << coverage.warning << ",\n";
  report << "    \"degraded\": " << coverage.degraded << ",\n";
  report << "    \"invalid\": " << coverage.invalid << ",\n";
  report << "    \"missing_prefix_count\": " << coverage.missing_prefix_count << ",\n";
  report << "    \"missing_suffix_count\": " << coverage.missing_suffix_count << ",\n";
  report << "    \"internal_gap_count\": " << coverage.internal_gap_count << ",\n";
  report << "    \"min_imu_count_in_window\": " << coverage.min_imu_count_in_window << ",\n";
  report << "    \"max_imu_count_in_window\": " << coverage.max_imu_count_in_window << ",\n";
  report << "    \"mean_imu_count_in_window\": " << coverage.mean_imu_count_in_window << ",\n";
  report << "    \"min_coverage_ratio\": " << coverage.min_coverage_ratio << ",\n";
  report << "    \"mean_coverage_ratio\": " << coverage.mean_coverage_ratio << ",\n";
  report << "    \"worst_reason\": ";
  WriteJsonString(report, coverage.worst_reason);
  report << ",\n";
  report << "    \"worst_sample\": {\n";
  report << "      \"available\": " << (coverage.has_worst_sample ? "true" : "false") << ",\n";
  report << "      \"scan_index\": " << coverage.worst_scan_index << ",\n";
  report << "      \"scan_start_ns\": " << coverage.worst_scan_start_ns << ",\n";
  report << "      \"scan_end_ns\": " << coverage.worst_scan_end_ns << ",\n";
  report << "      \"has_imu_bounds\": " << (coverage.worst_has_imu_bounds ? "true" : "false") << ",\n";
  report << "      \"first_imu_in_window_ns\": " << coverage.worst_first_imu_in_window_ns << ",\n";
  report << "      \"last_imu_in_window_ns\": " << coverage.worst_last_imu_in_window_ns << ",\n";
  report << "      \"reason\": ";
  WriteJsonString(report, coverage.worst_sample_reason);
  report << ",\n";
  report << "      \"imu_count_in_window\": " << coverage.worst_imu_count_in_window << ",\n";
  report << "      \"missing_prefix_ms\": " << coverage.worst_missing_prefix_ms << ",\n";
  report << "      \"missing_suffix_ms\": " << coverage.worst_missing_suffix_ms << ",\n";
  report << "      \"max_gap_inside_ms\": " << coverage.worst_max_gap_inside_ms << ",\n";
  report << "      \"coverage_ratio\": " << coverage.worst_coverage_ratio << "\n";
  report << "    }\n";
  report << "  },\n";

  report << "  \"fault_reasons\": {\n";
  std::size_t fault_index = 0;
  for (const auto& [reason, count] : coverage.fault_reasons) {
    report << "    ";
    WriteJsonString(report, reason);
    report << ": " << count;
    if (++fault_index < coverage.fault_reasons.size()) {
      report << ",";
    }
    report << "\n";
  }
  report << "  },\n";

  report << "  \"topics\": [\n";
  for (std::size_t i = 0; i < summary.topics.size(); ++i) {
    const auto& topic = summary.topics[i];
    report << "    {\n";
    report << "      \"name\": ";
    WriteJsonString(report, topic.name);
    report << ",\n";
    report << "      \"type\": ";
    WriteJsonString(report, topic.type);
    report << ",\n";
    report << "      \"message_count\": " << topic.message_count << "\n";
    report << "    }";
    if (i + 1 < summary.topics.size()) {
      report << ",";
    }
    report << "\n";
  }
  report << "  ]\n";
  report << "}\n";

  return true;
}

void PrintConsoleSummary(const AnalyzeBagOptions& options, const BagSummary& summary, std::ostream& out) {
  out << "Bag analyzed successfully.\n";
  out << "  bag: " << options.bag_path << "\n";
  out << "  report: " << options.report_path << "\n\n";

  out << "Selected topics:\n";
  out << "  lidar: " << options.lidar_topic << " messages=" << summary.lidar_messages
      << " found=" << (summary.lidar_topic_found ? "true" : "false") << "\n";
  out << "  imu: " << options.imu_topic << " messages=" << summary.imu_messages << " found=" << (summary.imu_topic_found ? "true" : "false")
      << "\n\n";

  out << "Verdict:\n";
  out << "  health=" << summary.verdict.health << "\n";
  out << "  reason=" << summary.verdict.reason << "\n";
  out << "  fault_ratio=" << summary.verdict.fault_ratio << "\n";
  out << "  map_update_recommended=" << (summary.verdict.map_update_recommended ? "true" : "false") << "\n\n";

  out << "IMU timing:\n";
  out << "  deserialized_messages=" << summary.imu_timing.deserialized_messages << "\n";
  out << "  deserialization_failures=" << summary.imu_timing.deserialization_failures << "\n";
  out << "  first_header_stamp_ns=" << summary.imu_timing.first_header_stamp_ns << "\n";
  out << "  last_header_stamp_ns=" << summary.imu_timing.last_header_stamp_ns << "\n";
  out << "  period_mean_ms=" << summary.imu_timing.period_mean_ms << "\n";
  out << "  period_min_ms=" << summary.imu_timing.period_min_ms << "\n";
  out << "  period_max_ms=" << summary.imu_timing.period_max_ms << "\n";
  out << "  period_stddev_ms=" << summary.imu_timing.period_stddev_ms << "\n";
  out << "  reorder_count=" << summary.imu_timing.reorder_count << "\n";

  out << "LiDAR timing:\n";
  out << "  deserialized_messages=" << summary.lidar_timing.deserialized_messages << "\n";
  out << "  deserialization_failures=" << summary.lidar_timing.deserialization_failures << "\n";
  out << "  first_header_stamp_ns=" << summary.lidar_timing.first_header_stamp_ns << "\n";
  out << "  last_header_stamp_ns=" << summary.lidar_timing.last_header_stamp_ns << "\n";
  out << "  period_mean_ms=" << summary.lidar_timing.period_mean_ms << "\n";
  out << "  period_min_ms=" << summary.lidar_timing.period_min_ms << "\n";
  out << "  period_max_ms=" << summary.lidar_timing.period_max_ms << "\n";
  out << "  period_stddev_ms=" << summary.lidar_timing.period_stddev_ms << "\n";
  out << "  reorder_count=" << summary.lidar_timing.reorder_count << "\n";

  out << "PointCloud2 capabilities:\n";
  out << "  has_xyz="
      << ((summary.lidar_first_cloud.capabilities.has_x && summary.lidar_first_cloud.capabilities.has_y &&
           summary.lidar_first_cloud.capabilities.has_z)
              ? "true"
              : "false")
      << "\n";
  out << "  has_intensity=" << (summary.lidar_first_cloud.capabilities.has_intensity ? "true" : "false") << "\n";
  out << "  has_ring=" << (summary.lidar_first_cloud.capabilities.has_ring ? "true" : "false") << "\n";
  out << "  time_field_name=" << summary.lidar_first_cloud.capabilities.time_field_name << "\n";
  out << "  time_field_datatype=" << summary.lidar_first_cloud.capabilities.time_field_datatype << "\n";
  out << "  point_time_role=" << summary.lidar_first_cloud.capabilities.point_time_role << "\n";
  out << "  point_time_supported=" << (summary.lidar_first_cloud.capabilities.point_time_supported ? "true" : "false") << "\n";
  out << "  reason=" << summary.lidar_first_cloud.capabilities.reason << "\n\n";

  out << "LiDAR scan windows:\n";
  out << "  scans_total=" << summary.lidar_scan_windows.scans_total << "\n";
  out << "  estimated=" << summary.lidar_scan_windows.estimated << "\n";
  out << "  with_point_time=" << summary.lidar_scan_windows.with_point_time << "\n";
  out << "  failed=" << summary.lidar_scan_windows.failed << "\n";
  out << "  duration_mean_ms=" << summary.lidar_scan_windows.duration_mean_ms << "\n";
  out << "  duration_min_ms=" << summary.lidar_scan_windows.duration_min_ms << "\n";
  out << "  duration_max_ms=" << summary.lidar_scan_windows.duration_max_ms << "\n";
  out << "  source=" << summary.lidar_scan_windows.source << "\n";
  out << "  confidence=" << summary.lidar_scan_windows.confidence << "\n";
  out << "  time_unit=" << summary.lidar_scan_windows.time_unit << "\n";
  out << "  last_reason=" << summary.lidar_scan_windows.last_reason << "\n\n";

  out << "Offline IMU coverage:\n";
  out << "  scans_total=" << summary.imu_coverage.scans_total << "\n";
  out << "  ok=" << summary.imu_coverage.ok << "\n";
  out << "  warning=" << summary.imu_coverage.warning << "\n";
  out << "  degraded=" << summary.imu_coverage.degraded << "\n";
  out << "  invalid=" << summary.imu_coverage.invalid << "\n";
  out << "  missing_prefix_count=" << summary.imu_coverage.missing_prefix_count << "\n";
  out << "  missing_suffix_count=" << summary.imu_coverage.missing_suffix_count << "\n";
  out << "  internal_gap_count=" << summary.imu_coverage.internal_gap_count << "\n";
  out << "  mean_imu_count_in_window=" << summary.imu_coverage.mean_imu_count_in_window << "\n";
  out << "  min_coverage_ratio=" << summary.imu_coverage.min_coverage_ratio << "\n";
  out << "  mean_coverage_ratio=" << summary.imu_coverage.mean_coverage_ratio << "\n";
  out << "  worst_reason=" << summary.imu_coverage.worst_reason << "\n\n";

  out << "Worst IMU coverage sample:\n";
  if (!summary.imu_coverage.has_worst_sample) {
    out << "  none\n\n";
  } else {
    out << "  scan_index=" << summary.imu_coverage.worst_scan_index << "\n";
    out << "  scan_start_ns=" << summary.imu_coverage.worst_scan_start_ns << "\n";
    out << "  scan_end_ns=" << summary.imu_coverage.worst_scan_end_ns << "\n";
    out << "  has_imu_bounds=" << (summary.imu_coverage.worst_has_imu_bounds ? "true" : "false") << "\n";
    out << "  first_imu_in_window_ns=" << summary.imu_coverage.worst_first_imu_in_window_ns << "\n";
    out << "  last_imu_in_window_ns=" << summary.imu_coverage.worst_last_imu_in_window_ns << "\n";
    out << "  reason=" << summary.imu_coverage.worst_sample_reason << "\n";
    out << "  imu_count_in_window=" << summary.imu_coverage.worst_imu_count_in_window << "\n";
    out << "  missing_prefix_ms=" << summary.imu_coverage.worst_missing_prefix_ms << "\n";
    out << "  missing_suffix_ms=" << summary.imu_coverage.worst_missing_suffix_ms << "\n";
    out << "  max_gap_inside_ms=" << summary.imu_coverage.worst_max_gap_inside_ms << "\n";
    out << "  coverage_ratio=" << summary.imu_coverage.worst_coverage_ratio << "\n\n";
  }

  out << "Fault reasons:\n";
  if (summary.imu_coverage.fault_reasons.empty()) {
    out << "  none\n";
  } else {
    for (const auto& [reason, count] : summary.imu_coverage.fault_reasons) {
      out << "  " << reason << "=" << count << "\n";
    }
  }
  out << "\n";

  out << "Topics:\n";
  for (const auto& topic : summary.topics) {
    out << "  " << topic.name << " " << topic.type << " messages=" << topic.message_count << "\n";
  }
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
    summary.verdict = BuildDatasetVerdict(summary);

    if (!WriteReportJson(options, summary, err)) {
      return 3;
    }

    PrintConsoleSummary(options, summary, out);
    return 0;
  } catch (const std::exception& e) {
    err << "Failed to analyze rosbag2.\n"
        << "  bag: " << options.bag_path << "\n"
        << "  error: " << e.what() << "\n";
    return 2;
  }
}

}  // namespace

int causal_slam::apps::ros2::RunAnalyzeBagCli(int argc, char** argv, std::ostream& out, std::ostream& err) {
  const auto options = ParseArgs(argc, argv, err);
  if (!options) {
    return 2;
  }

  if (options->show_help) {
    PrintUsage(out);
    return 0;
  }

  return AnalyzeBag(*options, out, err);
}