#include "pipeline/temporal_monitor_pipeline.h"

#include <algorithm>
#include <cstdint>
#include <utility>

namespace causal_slam::pipeline {
namespace {

constexpr double kNanosecondsPerMillisecond = 1'000'000.0;

double NanosecondsToMilliseconds(std::int64_t nanoseconds) {
  return static_cast<double>(nanoseconds) / kNanosecondsPerMillisecond;
}

std::vector<causal_slam::telemetry::StreamTimingDiagnostic> BuildTimingDiagnostics(
    const causal_slam::telemetry::TimingSummary& imu_summary,
    const causal_slam::telemetry::TimingSummary& lidar_summary) {
  return {
      causal_slam::telemetry::MakeStreamTimingDiagnostic(
          causal_slam::telemetry::TemporalStreamId::kImu, imu_summary),
      causal_slam::telemetry::MakeStreamTimingDiagnostic(
          causal_slam::telemetry::TemporalStreamId::kLidar, lidar_summary),
  };
}

}  // namespace

TemporalMonitorPipeline::TemporalMonitorPipeline(
    TemporalMonitorPipelineConfig config)
    : config_(config),
      imu_sample_buffer_(std::max<std::int64_t>(
          config_.imu_buffer_retention_ns, 100'000'000LL)),
      transform_age_analyzer_(config_.transform_age),
      temporal_statistics_(config_.statistics) {
  imu_timing_tracker_.SetGapThresholdMs(
      std::max(config_.imu_gap_threshold_ms, 1.0));
  lidar_timing_tracker_.SetGapThresholdMs(
      std::max(config_.lidar_gap_threshold_ms, 1.0));

  lidar_scan_window_estimator_.SetConfig(config_.lidar_scan_window);
  imu_coverage_analyzer_.SetConfig(config_.imu_coverage);
}

void TemporalMonitorPipeline::ObserveImu(const ImuPipelineInput& input) {
  imu_timing_tracker_.Observe(causal_slam::telemetry::TimingSample{
      .header_stamp_ns = input.header_stamp_ns,
      .receive_time_ns = input.receive_time_ns,
  });

  imu_sample_buffer_.Add(causal_slam::coverage::ImuSample{
      .stamp_ns = input.header_stamp_ns,
  });
}

void TemporalMonitorPipeline::ObserveLidar(const LidarPipelineInput& input) {
  latest_lidar_header_stamp_ns_ = input.header_stamp_ns;
  latest_lidar_frame_id_ = input.frame_id;

  if (!latest_lidar_point_time_diagnostics_.has_value()) {
    const auto inspection = point_cloud2_field_inspector_.Inspect(input.fields);
    latest_lidar_point_time_diagnostics_ = BuildPointTimeDiagnostics(inspection);

    if (inspection.primary_time_field.has_value()) {
      lidar_point_time_field_ = *inspection.primary_time_field;
    }
  }

  lidar_timing_tracker_.Observe(causal_slam::telemetry::TimingSample{
      .header_stamp_ns = input.header_stamp_ns,
      .receive_time_ns = input.receive_time_ns,
  });

  latest_lidar_scan_window_estimate_ =
      lidar_scan_window_estimator_.Estimate(input.header_stamp_ns);

  if (lidar_point_time_field_.has_value()) {
    const auto extraction =
        point_cloud2_time_field_extractor_.Extract(
            input.cloud_view, *lidar_point_time_field_);

    auto& point_time_diagnostics =
        latest_lidar_point_time_diagnostics_.emplace(
            latest_lidar_point_time_diagnostics_.value_or(
                causal_slam::model::PointTimeDiagnostics{}));

    point_time_diagnostics.extraction_attempted = true;
    point_time_diagnostics.extraction_used = extraction.has_scan_window;
    point_time_diagnostics.extraction_reason = extraction.reason;
    point_time_diagnostics.extraction_unit =
        causal_slam::pointcloud::ToString(extraction.time_unit);

    if (extraction.has_scan_window) {
      latest_lidar_scan_window_estimate_ =
          BuildPointTimeFieldEstimate(extraction);
    }
  }

  if (latest_lidar_scan_window_estimate_.has_value()) {
    latest_imu_coverage_summary_ = imu_coverage_analyzer_.Analyze(
        latest_lidar_scan_window_estimate_->window, imu_sample_buffer_.Samples());
  }
}

void TemporalMonitorPipeline::ObserveTransform(
    const causal_slam::transform::TransformLookupObservation& input) {
  latest_transform_age_summaries_.clear();
  latest_transform_age_summaries_.push_back(
      transform_age_analyzer_.Analyze(input));
}

void TemporalMonitorPipeline::ObserveTransforms(
    const std::vector<causal_slam::transform::TransformLookupObservation>& inputs) {
  latest_transform_age_summaries_.clear();
  latest_transform_age_summaries_.reserve(inputs.size());

  for (const auto& input : inputs) {
    latest_transform_age_summaries_.push_back(
        transform_age_analyzer_.Analyze(input));
  }
}

void TemporalMonitorPipeline::ObserveCloudDecision(
    const causal_slam::statistics::CloudDecisionEvent& event) {
  temporal_statistics_.ObserveCloudDecision(event);
}

causal_slam::diagnostics::TemporalDiagnosticSnapshot
TemporalMonitorPipeline::BuildLatestDiagnosticSnapshot() const {
  const causal_slam::model::TemporalObservation observation{
      .has_lidar_scan = latest_lidar_scan_window_estimate_.has_value(),
      .latest_lidar_header_stamp_ns = latest_lidar_header_stamp_ns_,
      .latest_lidar_frame_id = latest_lidar_frame_id_,
      .streams = BuildTimingDiagnostics(
          imu_timing_tracker_.CurrentWindowSummary(),
          lidar_timing_tracker_.CurrentWindowSummary()),
      .imu_coverage = latest_imu_coverage_summary_,
      .lidar_scan_window = latest_lidar_scan_window_estimate_,
      .lidar_point_time = latest_lidar_point_time_diagnostics_,
      .transform_ages = latest_transform_age_summaries_,
      .imu_buffer_size = imu_sample_buffer_.Size(),
  };

  const causal_slam::diagnostics::TemporalDiagnosticsBuilder diagnostics_builder;
  return diagnostics_builder.Build(observation);
}

TemporalMonitorPipelineSnapshot TemporalMonitorPipeline::BuildSnapshot(
    std::int64_t now_ns) {
  const auto imu_summary = imu_timing_tracker_.ConsumeWindowSummary();
  const auto lidar_summary = lidar_timing_tracker_.ConsumeWindowSummary();

  const auto streams = BuildTimingDiagnostics(imu_summary, lidar_summary);

  const causal_slam::model::TemporalObservation observation{
      .has_lidar_scan = latest_lidar_scan_window_estimate_.has_value(),
      .latest_lidar_header_stamp_ns = latest_lidar_header_stamp_ns_,
      .latest_lidar_frame_id = latest_lidar_frame_id_,
      .streams = streams,
      .imu_coverage = latest_imu_coverage_summary_,
      .lidar_scan_window = latest_lidar_scan_window_estimate_,
      .lidar_point_time = latest_lidar_point_time_diagnostics_,
      .transform_ages = latest_transform_age_summaries_,
      .imu_buffer_size = imu_sample_buffer_.Size(),
  };

  const causal_slam::diagnostics::TemporalDiagnosticsBuilder diagnostics_builder;
  const auto diagnostic_snapshot = diagnostics_builder.Build(observation);

  temporal_statistics_.Observe(
      now_ns, diagnostic_snapshot.observation, diagnostic_snapshot.overall_status);

  return TemporalMonitorPipelineSnapshot{
      .diagnostics = diagnostic_snapshot,
      .statistics = temporal_statistics_.Snapshot(now_ns),
  };
}

std::size_t TemporalMonitorPipeline::ImuBufferSize() const {
  return imu_sample_buffer_.Size();
}

causal_slam::model::PointTimeDiagnostics
TemporalMonitorPipeline::BuildPointTimeDiagnostics(
    const causal_slam::pointcloud::PointCloud2FieldInspection& inspection) const {
  causal_slam::model::PointTimeDiagnostics diagnostics;

  diagnostics.has_time_candidate = inspection.has_time_candidate;
  diagnostics.has_supported_time_field = inspection.has_supported_time_field;
  diagnostics.inspection_reason = inspection.reason;

  const causal_slam::pointcloud::PointCloud2FieldInfo* diagnostic_field = nullptr;

  if (inspection.primary_time_field.has_value()) {
    diagnostic_field = &*inspection.primary_time_field;
  } else {
    for (const auto& field : inspection.fields) {
      if (field.time_role != causal_slam::pointcloud::PointCloud2TimeFieldRole::kNone) {
        diagnostic_field = &field;
        break;
      }
    }
  }

  if (diagnostic_field != nullptr) {
    diagnostics.field_name = diagnostic_field->name;
    diagnostics.field_datatype =
        causal_slam::pointcloud::PointCloud2DatatypeToString(
            diagnostic_field->datatype);
    diagnostics.field_role =
        causal_slam::pointcloud::ToString(diagnostic_field->time_role);
  }

  return diagnostics;
}

causal_slam::lidar::LidarScanWindowEstimate
TemporalMonitorPipeline::BuildPointTimeFieldEstimate(
    const causal_slam::pointcloud::PointCloud2TimeFieldExtraction& extraction) const {
  return causal_slam::lidar::LidarScanWindowEstimate{
      .window = extraction.scan_window,
      .duration_ms =
          NanosecondsToMilliseconds(extraction.scan_window.DurationNs()),
      .source = causal_slam::lidar::LidarScanWindowSource::kPointTimeField,
      .confidence = causal_slam::lidar::LidarScanWindowConfidence::kHigh,
      .reason = extraction.reason + ":" +
                causal_slam::pointcloud::ToString(extraction.time_unit),
  };
}

}  // namespace causal_slam::pipeline
