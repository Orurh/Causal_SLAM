#include "temporal_report_builder.h"

#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "domain/policy/map_update_decision.h"
#include "domain/sensors/imu/imu_coverage_analyzer.h"
#include "domain/sensors/lidar/lidar_scan_window_estimator.h"
#include "domain/sensors/transform/transform_age_analyzer.h"
#include "domain/telemetry/stream_timing_tracker.h"

namespace causal_slam::report {
namespace {

std::string BoolString(bool value) {
  return value ? "true" : "false";
}

std::string JoinStrings(const std::vector<std::string>& values, std::string_view separator) {
  std::ostringstream out;

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << separator;
    }
    out << values[i];
  }

  return out.str();
}

template <typename T>
std::string ToReportString(const T& value) {
  std::ostringstream out;
  out << value;
  return out.str();
}

ReportMetric Metric(std::string name, std::string value) {
  return ReportMetric{.name = std::move(name), .value = std::move(value)};
}

ReportSection MakeSection(std::string id, std::string title, std::string empty_message = "none") {
  ReportSection section;
  section.id = std::move(id);
  section.title = std::move(title);
  section.empty_message = std::move(empty_message);
  return section;
}

ReportRow MakeMetricRow(std::string label, std::string status, std::vector<ReportMetric> metrics = {}, std::string reason = {},
                        std::string detail = {}) {
  ReportRow row;
  row.label = std::move(label);
  row.status = std::move(status);
  row.metrics = std::move(metrics);
  row.reason = std::move(reason);
  row.detail = std::move(detail);
  return row;
}

ReportSection BuildHealthSection(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  auto section = MakeSection("temporal_health", "Temporal Health");
  section.metrics.push_back(Metric("status", causal_slam::telemetry::ToString(snapshot.overall_status)));
  return section;
}

ReportSection BuildMapUpdateSection(const causal_slam::policy::MapUpdateDecision& map_update_decision) {
  auto section = MakeSection("map_update", "Map update decision");

  section.metrics.push_back(Metric("allowed", BoolString(map_update_decision.map_update_allowed)));
  section.metrics.push_back(Metric("reason", causal_slam::policy::ToString(map_update_decision.reason)));

  return section;
}

ReportSection BuildLidarScanWindowSection(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  auto section = MakeSection("lidar_scan_window", "LiDAR scan window");

  if (!snapshot.observation.lidar_scan_window.has_value()) {
    section.metrics.push_back(Metric("status", "not_available"));
    section.metrics.push_back(Metric("reason", "no_lidar_scan_received_yet"));
    return section;
  }

  const auto& scan_window = *snapshot.observation.lidar_scan_window;

  section.metrics.push_back(Metric("source", causal_slam::lidar::ToString(scan_window.source)));
  section.metrics.push_back(Metric("confidence", causal_slam::lidar::ToString(scan_window.confidence)));
  section.metrics.push_back(Metric("duration_ms", ToReportString(scan_window.duration_ms)));
  section.metrics.push_back(Metric("reason", scan_window.reason));

  return section;
}

ReportSection BuildImuCoverageSection(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  auto section = MakeSection("imu_coverage", "IMU coverage");

  if (!snapshot.observation.imu_coverage.has_value()) {
    section.metrics.push_back(Metric("status", "not_available"));
    section.metrics.push_back(Metric("reason", "no_lidar_scan_received_yet"));
    return section;
  }

  const auto& coverage = *snapshot.observation.imu_coverage;

  section.metrics.push_back(Metric("status", causal_slam::coverage::ToString(coverage.health)));
  section.metrics.push_back(Metric("samples_in_window", ToReportString(coverage.imu_count_in_window)));
  section.metrics.push_back(Metric("coverage_ratio", ToReportString(coverage.coverage_ratio)));
  section.metrics.push_back(Metric("max_gap_inside_ms", ToReportString(coverage.max_gap_inside_ms)));

  return section;
}

ReportSection BuildStreamsSection(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  auto section = MakeSection("streams", "Streams");

  for (const auto& stream : snapshot.observation.streams) {
    const auto& summary = stream.timing;

    section.rows.push_back(MakeMetricRow(causal_slam::telemetry::ToString(stream.id), causal_slam::telemetry::ToString(summary.health),
                                         {
                                             Metric("period_ms", ToReportString(summary.last_period_ms)),
                                             Metric("jitter_ms", ToReportString(summary.last_jitter_ms)),
                                             Metric("gaps", ToReportString(summary.window_gap_count)),
                                             Metric("reordered", ToReportString(summary.window_reordered_count)),
                                         }));
  }

  return section;
}

ReportSection BuildTransformChecksSection(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  auto section = MakeSection("tf_checks", "TF checks");

  for (const auto& transform_age : snapshot.observation.transform_ages) {
    section.rows.push_back(MakeMetricRow(transform_age.target_frame + " <- " + transform_age.source_frame,
                                         causal_slam::telemetry::ToString(transform_age.health),
                                         {
                                             Metric("status", causal_slam::transform::ToString(transform_age.status)),
                                             Metric("age_ms", ToReportString(transform_age.transform_age_ms)),
                                             Metric("receive_delay_ms", ToReportString(transform_age.receive_delay_ms)),
                                         },
                                         transform_age.reason, transform_age.adapter_detail));
  }

  return section;
}

void MaybeAppendLidarPointTimeSection(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot, ReportDocument* document) {
  if (!snapshot.observation.lidar_point_time.has_value() || snapshot.observation.lidar_point_time->inspection_reason.empty()) {
    return;
  }

  const auto& point_time = *snapshot.observation.lidar_point_time;

  auto section = MakeSection("lidar_point_time", "LiDAR point time");
  section.metrics.push_back(Metric("candidate", BoolString(point_time.has_time_candidate)));
  section.metrics.push_back(Metric("supported", BoolString(point_time.has_supported_time_field)));
  section.metrics.push_back(Metric("field", point_time.field_name));
  section.metrics.push_back(Metric("datatype", point_time.field_datatype));
  section.metrics.push_back(Metric("reason", point_time.inspection_reason));

  document->sections.push_back(std::move(section));
}

ReportSection BuildIssuesSection(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) {
  auto section = MakeSection("issues", "Issues");

  for (const auto& issue : snapshot.issues) {
    ReportRow row;
    row.label = issue.title;
    row.status = causal_slam::diagnostics::ToString(issue.severity);
    row.reason = causal_slam::diagnostics::ToString(issue.reason);
    row.explanation = issue.explanation;
    row.evidence = issue.evidence;
    row.suggested_action = issue.suggested_action;

    section.rows.push_back(std::move(row));
  }

  return section;
}

void RenderNumericStatsAsMetrics(const causal_slam::statistics::NumericStats& stats, std::vector<ReportMetric>* metrics) {
  if (stats.count == 0) {
    metrics->push_back(Metric("count", "0"));
    metrics->push_back(Metric("value", "n/a"));
    return;
  }

  metrics->push_back(Metric("count", ToReportString(stats.count)));
  metrics->push_back(Metric("mean", ToReportString(stats.mean)));
  metrics->push_back(Metric("median", ToReportString(stats.median)));
  metrics->push_back(Metric("p95", ToReportString(stats.p95)));
  metrics->push_back(Metric("max", ToReportString(stats.max)));
}

ReportRow BuildNumericStatsRow(std::string label, const causal_slam::statistics::NumericStats& stats) {
  ReportRow row;
  row.label = std::move(label);
  row.status = "";

  RenderNumericStatsAsMetrics(stats, &row.metrics);

  return row;
}

void AppendStreamTimingStatisticsRows(const causal_slam::statistics::TemporalWindowStatistics& stats, ReportSection* section) {
  if (stats.streams.empty()) {
    section->rows.push_back(MakeMetricRow("streams", "none"));
    return;
  }

  for (const auto& stream : stats.streams) {
    section->rows.push_back(MakeMetricRow(causal_slam::telemetry::ToString(stream.id), "delay_ms",
                                          {
                                              Metric("count", ToReportString(stream.delay_ms.count)),
                                              Metric("mean", ToReportString(stream.delay_ms.mean)),
                                              Metric("median", ToReportString(stream.delay_ms.median)),
                                              Metric("p95", ToReportString(stream.delay_ms.p95)),
                                              Metric("max", ToReportString(stream.delay_ms.max)),
                                          }));

    section->rows.push_back(MakeMetricRow(causal_slam::telemetry::ToString(stream.id), "period_ms",
                                          {
                                              Metric("count", ToReportString(stream.period_ms.count)),
                                              Metric("mean", ToReportString(stream.period_ms.mean)),
                                              Metric("median", ToReportString(stream.period_ms.median)),
                                              Metric("p95", ToReportString(stream.period_ms.p95)),
                                              Metric("max", ToReportString(stream.period_ms.max)),
                                          }));

    section->rows.push_back(MakeMetricRow(causal_slam::telemetry::ToString(stream.id), "jitter_ms",
                                          {
                                              Metric("count", ToReportString(stream.jitter_ms.count)),
                                              Metric("mean", ToReportString(stream.jitter_ms.mean)),
                                              Metric("median", ToReportString(stream.jitter_ms.median)),
                                              Metric("p95", ToReportString(stream.jitter_ms.p95)),
                                              Metric("max", ToReportString(stream.jitter_ms.max)),
                                          }));
  }
}

ReportSection BuildCloudDecisionsSection(const causal_slam::statistics::CloudDecisionStatistics& stats) {
  auto section = MakeSection("cloud_decisions", "Cloud decisions", "no cloud decisions observed");

  section.metrics.push_back(Metric("total", ToReportString(stats.total_count)));
  section.metrics.push_back(Metric("forwarded", ToReportString(stats.forwarded_count)));
  section.metrics.push_back(Metric("blocked", ToReportString(stats.blocked_count)));
  section.metrics.push_back(Metric("blocked_warmup", ToReportString(stats.blocked_warmup_count)));
  section.metrics.push_back(Metric("blocked_by_gate", ToReportString(stats.blocked_by_gate_count)));

  for (const auto& reason : stats.block_reasons) {
    section.rows.push_back(MakeMetricRow(reason.reason, "blocked",
                                         {
                                             Metric("count", ToReportString(reason.count)),
                                         }));
  }

  for (const auto& event : stats.recent_blocked_events) {
    std::vector<ReportMetric> metrics{
        Metric("sequence", ToReportString(event.sequence_id)),
        Metric("stamp_ns", ToReportString(event.header_stamp_ns)),
        Metric("frame_id", event.frame_id),
        Metric("points", ToReportString(event.point_count)),
        Metric("bytes", ToReportString(event.data_size_bytes)),
        Metric("decision", causal_slam::statistics::ToString(event.decision)),
        Metric("map_update_allowed", BoolString(event.map_update_allowed)),
    };

    if (event.has_scan_window) {
      metrics.push_back(Metric("scan_duration_ms", ToReportString(event.scan_duration_ms)));
      metrics.push_back(Metric("scan_source", event.scan_window_source));
      metrics.push_back(Metric("scan_confidence", event.scan_window_confidence));
    }

    if (event.has_imu_coverage) {
      metrics.push_back(Metric("imu_samples", ToReportString(event.imu_samples_in_window)));
      metrics.push_back(Metric("imu_coverage", ToReportString(event.imu_coverage_ratio)));
      metrics.push_back(Metric("imu_max_gap_ms", ToReportString(event.imu_max_gap_inside_ms)));
    }

    auto row = MakeMetricRow("blocked cloud #" + ToReportString(event.sequence_id), causal_slam::telemetry::ToString(event.health),
                             std::move(metrics), event.reason, JoinStrings(event.fault_reasons, ", "));
    row.collapsed = true;
    section.rows.push_back(std::move(row));
  }

  return section;
}

ReportSection BuildStatisticsWindowSection(std::string id, std::string title,
                                           const causal_slam::statistics::TemporalWindowStatistics& stats) {
  auto section = MakeSection(std::move(id), std::move(title));

  section.metrics.push_back(Metric("samples", ToReportString(stats.sample_count)));

  section.rows.push_back(MakeMetricRow("health", "",
                                       {
                                           Metric("OK", ToReportString(stats.health.ok_count)),
                                           Metric("WARNING", ToReportString(stats.health.warning_count)),
                                           Metric("DEGRADED", ToReportString(stats.health.degraded_count)),
                                           Metric("INVALID", ToReportString(stats.health.invalid_count)),
                                       }));

  section.rows.push_back(
      MakeMetricRow("scan_window_sources", "",
                    {
                        Metric("point_time_field", ToReportString(stats.scan_window_sources.point_time_field_count)),
                        Metric("measured_header_period", ToReportString(stats.scan_window_sources.measured_header_period_count)),
                        Metric("assumed_fixed_duration", ToReportString(stats.scan_window_sources.assumed_fixed_duration_count)),
                        Metric("driver_metadata", ToReportString(stats.scan_window_sources.driver_metadata_count)),
                    }));

  section.rows.push_back(MakeMetricRow("scan_window_confidence", "",
                                       {
                                           Metric("HIGH", ToReportString(stats.scan_window_confidence.high_count)),
                                           Metric("MEDIUM", ToReportString(stats.scan_window_confidence.medium_count)),
                                           Metric("LOW", ToReportString(stats.scan_window_confidence.low_count)),
                                       }));

  AppendStreamTimingStatisticsRows(stats, &section);

  section.rows.push_back(BuildNumericStatsRow("imu_coverage_ratio", stats.imu_coverage_ratio));
  section.rows.push_back(BuildNumericStatsRow("imu_samples_in_window", stats.imu_samples_in_window));
  section.rows.push_back(BuildNumericStatsRow("imu_max_gap_inside_ms", stats.imu_max_gap_inside_ms));

  return section;
}

}  // namespace

ReportDocument TemporalReportBuilder::BuildDiagnosticsReport(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                                             const causal_slam::policy::MapUpdateDecision& map_update_decision) const {
  ReportDocument document;
  document.title = "";

  document.sections.push_back(BuildHealthSection(snapshot));
  document.sections.push_back(BuildMapUpdateSection(map_update_decision));
  document.sections.push_back(BuildLidarScanWindowSection(snapshot));
  document.sections.push_back(BuildImuCoverageSection(snapshot));
  document.sections.push_back(BuildStreamsSection(snapshot));
  document.sections.push_back(BuildTransformChecksSection(snapshot));

  MaybeAppendLidarPointTimeSection(snapshot, &document);

  document.sections.push_back(BuildIssuesSection(snapshot));

  return document;
}

ReportDocument TemporalReportBuilder::BuildStatisticsReport(const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const {
  ReportDocument document;
  document.title = "Temporal Statistics (historical windows)";

  struct WindowView {
    std::string_view id;
    std::string_view title;
    const causal_slam::statistics::TemporalWindowStatistics* stats;
  };

  const std::vector<WindowView> windows{
      {.id = "last_10s", .title = "Last 10s", .stats = &snapshot.short_window},
      {.id = "last_60s", .title = "Last 60s", .stats = &snapshot.medium_window},
      {.id = "session", .title = "Session", .stats = &snapshot.session},
  };

  document.sections.push_back(BuildCloudDecisionsSection(snapshot.cloud_decisions));

  for (const auto& window : windows) {
    document.sections.push_back(BuildStatisticsWindowSection(std::string{window.id}, std::string{window.title}, *window.stats));
  }

  return document;
}

}  // namespace causal_slam::report
