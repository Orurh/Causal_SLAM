#include "offline_temporal_report_json_renderer.h"

#include <cstdint>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace causal_slam::render {
namespace {

using causal_slam::offline_analysis::TimingSummary;

double NsToMs(std::int64_t ns) {
  return static_cast<double>(ns) / 1'000'000.0;
}

std::string JsonEscape(std::string_view value) {
  std::ostringstream escaped;

  for (const char ch : value) {
    switch (ch) {
      case '"':
        escaped << "\\\"";
        break;
      case '\\':
        escaped << "\\\\";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(ch));
        } else {
          escaped << ch;
        }
        break;
    }
  }

  return escaped.str();
}

void WriteJsonString(std::ostream& out, std::string_view value) {
  out << '"' << JsonEscape(value) << '"';
}

void WriteNullableDouble(std::ostream& out, bool has_value, double value) {
  if (!has_value) {
    out << "null";
    return;
  }

  out << value;
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

}  // namespace

std::string OfflineTemporalReportJsonRenderer::Render(const OfflineTemporalReportRenderContext& context,
                                                      const causal_slam::offline_analysis::OfflineTemporalReport& summary) const {
  std::ostringstream report;

  report << "{\n";
  report << "  \"tool\": \"causal_slam_analyze_bag\",\n";
  report << "  \"schema_version\": 1,\n";
  report << "  \"bag\": ";
  WriteJsonString(report, context.bag_path);
  report << ",\n";

  report << "  \"selected_topics\": {\n";
  report << "    \"lidar\": ";
  WriteJsonString(report, context.lidar_topic);
  report << ",\n";
  report << "    \"imu\": ";
  WriteJsonString(report, context.imu_topic);
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

  const auto& stream_timing = summary.stream_timing_faults;
  report << "  \"stream_timing_faults\": {\n";
  report << "    \"lidar_stream_timing_jitter_high\": " << (stream_timing.lidar_stream_timing_jitter_high ? "true" : "false") << ",\n";
  report << "    \"lidar_stream_timing_short_period\": " << (stream_timing.lidar_stream_timing_short_period ? "true" : "false") << ",\n";
  report << "    \"lidar_stream_timing_long_period\": " << (stream_timing.lidar_stream_timing_long_period ? "true" : "false") << ",\n";
  report << "    \"lidar_period_jitter_threshold_ms\": " << stream_timing.lidar_period_jitter_threshold_ms << ",\n";
  report << "    \"lidar_period_short_threshold_ratio\": " << stream_timing.lidar_period_short_threshold_ratio << ",\n";
  report << "    \"lidar_period_long_threshold_ratio\": " << stream_timing.lidar_period_long_threshold_ratio << ",\n";
  report << "    \"fault_reasons\": {\n";
  std::size_t stream_fault_index = 0;
  for (const auto& [reason, count] : stream_timing.fault_reasons) {
    report << "      ";
    WriteJsonString(report, reason);
    report << ": " << count;
    if (++stream_fault_index < stream_timing.fault_reasons.size()) {
      report << ",";
    }
    report << "\n";
  }
  report << "    }\n";
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
  report << "  \"point_cloud2_capability\": {\n";
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

  report << "  \"point_cloud2_fields\": [\n";
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
  report << "    \"duration_outlier_count\": " << windows.duration_outlier_count << ",\n";
  report << "    \"duration_outlier_ratio\": " << windows.duration_outlier_ratio << ",\n";
  report << "    \"duration_outlier_threshold_ms\": " << windows.duration_outlier_threshold_ms << ",\n";
  report << "    \"worst_duration_outlier_scan_index\": " << windows.worst_duration_outlier_scan_index << ",\n";
  report << "    \"worst_duration_outlier_ms\": " << windows.worst_duration_outlier_ms << ",\n";
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

  return report.str();
}

}  // namespace causal_slam::render
