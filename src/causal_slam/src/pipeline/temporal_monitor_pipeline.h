#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "coverage/imu_coverage_analyzer.h"
#include "coverage/imu_sample_buffer.h"
#include "diagnostics/temporal_diagnostics.h"
#include "lidar/lidar_scan_window_estimator.h"
#include "model/point_time_diagnostics.h"
#include "pointcloud/point_cloud2_field_inspector.h"
#include "pointcloud/point_cloud2_time_field_extractor.h"
#include "statistics/temporal_statistics.h"
#include "telemetry/stream_timing_tracker.h"
#include "transform/transform_age_analyzer.h"
#include "transform/transform_lookup_observation.h"

namespace causal_slam::pipeline {

struct TemporalMonitorPipelineConfig {
  double imu_gap_threshold_ms{100.0};
  double lidar_gap_threshold_ms{500.0};

  std::int64_t imu_buffer_retention_ns{5'000'000'000LL};

  causal_slam::lidar::LidarScanWindowEstimatorConfig lidar_scan_window;
  causal_slam::coverage::ImuCoverageConfig imu_coverage;
  causal_slam::statistics::TemporalStatisticsAggregatorConfig statistics;
  causal_slam::transform::TransformAgeAnalyzerConfig transform_age;
};

struct ImuPipelineInput {
  std::int64_t header_stamp_ns{0};
  std::int64_t receive_time_ns{0};
};

struct LidarPipelineInput {
  std::int64_t header_stamp_ns{0};
  std::int64_t receive_time_ns{0};
  std::string frame_id;

  std::vector<causal_slam::pointcloud::PointCloud2FieldInfo> fields;
  causal_slam::pointcloud::PointCloud2CloudView cloud_view;
};

struct TemporalMonitorPipelineSnapshot {
  causal_slam::diagnostics::TemporalDiagnosticSnapshot diagnostics;
  causal_slam::statistics::TemporalStatisticsSnapshot statistics;
};

class TemporalMonitorPipeline final {
 public:
  explicit TemporalMonitorPipeline(
      TemporalMonitorPipelineConfig config = TemporalMonitorPipelineConfig{});

  void ObserveImu(const ImuPipelineInput& input);
  void ObserveLidar(const LidarPipelineInput& input);
  void ObserveTransform(
      const causal_slam::transform::TransformLookupObservation& input);
  void ObserveTransforms(
      const std::vector<causal_slam::transform::TransformLookupObservation>& inputs);

  [[nodiscard]] TemporalMonitorPipelineSnapshot BuildSnapshot(
      std::int64_t now_ns);

  [[nodiscard]] causal_slam::diagnostics::TemporalDiagnosticSnapshot
  BuildLatestDiagnosticSnapshot() const;

  [[nodiscard]] std::size_t ImuBufferSize() const;

 private:
  [[nodiscard]] causal_slam::model::PointTimeDiagnostics BuildPointTimeDiagnostics(
      const causal_slam::pointcloud::PointCloud2FieldInspection& inspection) const;

  [[nodiscard]] causal_slam::lidar::LidarScanWindowEstimate BuildPointTimeFieldEstimate(
      const causal_slam::pointcloud::PointCloud2TimeFieldExtraction& extraction) const;

  TemporalMonitorPipelineConfig config_;

  causal_slam::telemetry::StreamTimingTracker imu_timing_tracker_;
  causal_slam::telemetry::StreamTimingTracker lidar_timing_tracker_;

  causal_slam::lidar::LidarScanWindowEstimator lidar_scan_window_estimator_;
  std::optional<causal_slam::lidar::LidarScanWindowEstimate>
      latest_lidar_scan_window_estimate_;
  std::int64_t latest_lidar_header_stamp_ns_{0};
  std::string latest_lidar_frame_id_;

  causal_slam::coverage::ImuSampleBuffer imu_sample_buffer_;
  causal_slam::coverage::ImuCoverageAnalyzer imu_coverage_analyzer_;
  std::optional<causal_slam::coverage::ImuCoverageSummary>
      latest_imu_coverage_summary_;

  causal_slam::pointcloud::PointCloud2FieldInspector point_cloud2_field_inspector_;
  causal_slam::pointcloud::PointCloud2TimeFieldExtractor point_cloud2_time_field_extractor_;

  causal_slam::transform::TransformAgeAnalyzer transform_age_analyzer_;
  std::vector<causal_slam::transform::TransformAgeSummary>
      latest_transform_age_summaries_;

  std::optional<causal_slam::pointcloud::PointCloud2FieldInfo> lidar_point_time_field_;
  std::optional<causal_slam::model::PointTimeDiagnostics>
      latest_lidar_point_time_diagnostics_;

  causal_slam::statistics::TemporalStatisticsAggregator temporal_statistics_;
};

}  // namespace causal_slam::pipeline
