#include "domain/diagnostics/temporal_diagnostics.h"

#include "domain/policy/map_update_decision.h"

#include <algorithm>
#include <string>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

namespace causal_slam::diagnostics {
namespace {

namespace coverage = causal_slam::coverage;
namespace lidar = causal_slam::lidar;
namespace model = causal_slam::model;
namespace policy = causal_slam::policy;
namespace telemetry = causal_slam::telemetry;
namespace transform = causal_slam::transform;

template <typename T, typename = void>
struct HasOverallStatus : std::false_type {};

template <typename T>
struct HasOverallStatus<T, std::void_t<decltype(std::declval<T>().overall_status)>> : std::true_type {};

static_assert(!HasOverallStatus<model::TemporalObservation>::value);
static_assert(HasOverallStatus<TemporalDiagnosticSnapshot>::value);

telemetry::TimingSummary OkTiming() {
  telemetry::TimingSummary summary;
  summary.window_count = 10;
  summary.health = telemetry::TimingHealth::kOk;
  summary.reason = "ok";
  return summary;
}

coverage::ImuCoverageSummary OkCoverage() {
  coverage::ImuCoverageSummary summary;
  summary.imu_count_in_window = 5;
  summary.coverage_ratio = 0.8;
  summary.max_gap_inside_ms = 20.0;
  summary.health = coverage::ImuCoverageHealth::kOk;
  summary.reason = "ok";
  return summary;
}

coverage::ImuCoverageSummary DegradedCoverage() {
  coverage::ImuCoverageSummary summary;
  summary.imu_count_in_window = 0;
  summary.coverage_ratio = 0.0;
  summary.missing_prefix_ms = 100.0;
  summary.missing_suffix_ms = 100.0;
  summary.max_gap_inside_ms = 0.0;
  summary.health = coverage::ImuCoverageHealth::kDegraded;
  summary.reason = "imu_window_empty";
  return summary;
}

lidar::LidarScanWindowEstimate HighConfidencePointTimeWindow() {
  lidar::LidarScanWindowEstimate estimate;
  estimate.duration_ms = 100.0;
  estimate.source = lidar::LidarScanWindowSource::kPointTimeField;
  estimate.confidence = lidar::LidarScanWindowConfidence::kHigh;
  estimate.reason = "point_time_field_extracted:relative_nanoseconds";
  return estimate;
}

lidar::LidarScanWindowEstimate MediumConfidenceMeasuredWindow() {
  lidar::LidarScanWindowEstimate estimate;
  estimate.duration_ms = 100.0;
  estimate.source = lidar::LidarScanWindowSource::kMeasuredHeaderPeriod;
  estimate.confidence = lidar::LidarScanWindowConfidence::kMedium;
  estimate.reason = "measured_header_period";
  return estimate;
}

lidar::LidarScanWindowEstimate LowConfidenceFallbackWindow() {
  lidar::LidarScanWindowEstimate estimate;
  estimate.duration_ms = 100.0;
  estimate.source = lidar::LidarScanWindowSource::kAssumedFixedDuration;
  estimate.confidence = lidar::LidarScanWindowConfidence::kLow;
  estimate.reason = "no_previous_lidar_stamp";
  return estimate;
}

model::PointTimeDiagnostics SupportedOffsetTime() {
  model::PointTimeDiagnostics diagnostics;
  diagnostics.has_time_candidate = true;
  diagnostics.has_supported_time_field = true;
  diagnostics.field_name = "offset_time";
  diagnostics.field_datatype = "UINT32";
  diagnostics.field_role = "point_offset_time";
  diagnostics.inspection_reason = "supported_point_time_field_detected";
  diagnostics.extraction_attempted = true;
  diagnostics.extraction_used = true;
  diagnostics.extraction_reason = "point_time_field_extracted";
  diagnostics.extraction_unit = "relative_nanoseconds";
  return diagnostics;
}

transform::TransformAgeSummary OkTransformAge() {
  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kOk;
  summary.status = transform::TransformLookupStatus::kOk;
  summary.target_frame = "odom";
  summary.source_frame = "base_link";
  summary.transform_age_ms = 10.0;
  summary.receive_delay_ms = 5.0;
  summary.reason = "ok";
  return summary;
}

transform::TransformAgeSummary FailedTransformLookup() {
  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kInvalid;
  summary.status = transform::TransformLookupStatus::kLookupFailed;
  summary.target_frame = "odom";
  summary.source_frame = "base_link";
  summary.transform_age_ms = 0.0;
  summary.receive_delay_ms = 5.0;
  summary.reason = "tf_lookup_failed";
  summary.adapter_detail = "frame_not_found";
  return summary;
}

transform::TransformAgeSummary ExtrapolatedTransformLookup() {
  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kDegraded;
  summary.status = transform::TransformLookupStatus::kExtrapolationRequired;
  summary.target_frame = "odom";
  summary.source_frame = "base_link";
  summary.transform_age_ms = 0.0;
  summary.receive_delay_ms = 5.0;
  summary.reason = "tf_extrapolation_required";
  summary.adapter_detail = "extrapolation_into_future";
  return summary;
}

transform::TransformAgeSummary StaleTransformLookup() {
  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kDegraded;
  summary.status = transform::TransformLookupStatus::kTransformAgeTooHigh;
  summary.target_frame = "odom";
  summary.source_frame = "base_link";
  summary.transform_age_ms = 120.0;
  summary.receive_delay_ms = 5.0;
  summary.reason = "tf_age_too_high";
  return summary;
}

transform::TransformAgeSummary FutureTransformLookup() {
  transform::TransformAgeSummary summary;
  summary.health = telemetry::TemporalHealthStatus::kDegraded;
  summary.status = transform::TransformLookupStatus::kTransformFromFuture;
  summary.target_frame = "odom";
  summary.source_frame = "base_link";
  summary.transform_age_ms = -10.0;
  summary.receive_delay_ms = 5.0;
  summary.reason = "tf_transform_from_future";
  return summary;
}

model::PointTimeDiagnostics RejectedFloat32Timestamp() {
  model::PointTimeDiagnostics diagnostics;
  diagnostics.has_time_candidate = true;
  diagnostics.has_supported_time_field = false;
  diagnostics.field_name = "timestamp";
  diagnostics.field_datatype = "FLOAT32";
  diagnostics.field_role = "point_time";
  diagnostics.inspection_reason = "absolute_float32_timestamp_precision_unsafe";
  return diagnostics;
}

model::TemporalObservation BaseOkInput() {
  model::TemporalObservation input;
  input.streams = {
      telemetry::MakeStreamTimingDiagnostic(telemetry::TemporalStreamId::kImu, OkTiming()),
      telemetry::MakeStreamTimingDiagnostic(telemetry::TemporalStreamId::kLidar, OkTiming()),
  };
  input.imu_coverage = OkCoverage();
  input.lidar_scan_window = HighConfidencePointTimeWindow();
  input.lidar_point_time = SupportedOffsetTime();
  input.imu_buffer_size = 250;
  return input;
}

bool HasIssueWithTitle(const TemporalDiagnosticSnapshot& snapshot, const std::string& title) {
  return std::any_of(snapshot.issues.begin(), snapshot.issues.end(), [&](const auto& issue) { return issue.title == title; });
}

bool HasIssueWithReason(const TemporalDiagnosticSnapshot& snapshot, TemporalFaultReason reason) {
  return std::any_of(snapshot.issues.begin(), snapshot.issues.end(), [reason](const auto& issue) { return issue.reason == reason; });
}

TEST(TemporalDiagnosticsBuilderTest, FaultReasonToStringIsStable) {
  EXPECT_STREQ(ToString(TemporalFaultReason::kNone), "none");
  EXPECT_STREQ(ToString(TemporalFaultReason::kStreamTimingUnstable), "stream_timing_unstable");
  EXPECT_STREQ(ToString(TemporalFaultReason::kImuWindowIncomplete), "imu_window_incomplete");
  EXPECT_STREQ(ToString(TemporalFaultReason::kLidarPointTimeUnsupported), "lidar_point_time_unsupported");
  EXPECT_STREQ(ToString(TemporalFaultReason::kTfLookupFailed), "tf_lookup_failed");
  EXPECT_STREQ(ToString(TemporalFaultReason::kTfExtrapolationRequired), "tf_extrapolation_required");
  EXPECT_STREQ(ToString(TemporalFaultReason::kTfAgeTooHigh), "tf_age_too_high");
  EXPECT_STREQ(ToString(TemporalFaultReason::kTfTransformFromFuture), "tf_transform_from_future");
}

TEST(TemporalDiagnosticsBuilderTest, SupportedPointTimeAndOkCoverageProducesOkSnapshot) {
  const TemporalDiagnosticsBuilder builder;

  const auto snapshot = builder.Build(BaseOkInput());

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kOk);
  EXPECT_TRUE(snapshot.issues.empty());
}

TEST(TemporalDiagnosticsBuilderTest, RejectedFloat32TimestampProducesWarning) {
  auto input = BaseOkInput();
  input.lidar_scan_window = MediumConfidenceMeasuredWindow();
  input.lidar_point_time = RejectedFloat32Timestamp();

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kWarning);

  EXPECT_TRUE(HasIssueWithTitle(snapshot, "LiDAR point timestamps were detected but not trusted"));
  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kLidarPointTimeUnsupported));
}

TEST(TemporalDiagnosticsBuilderTest, MeasuredHeaderPeriodWithoutPointTimeCandidateIsOk) {
  auto input = BaseOkInput();
  input.lidar_scan_window = MediumConfidenceMeasuredWindow();

  model::PointTimeDiagnostics no_point_time;
  no_point_time.has_time_candidate = false;
  no_point_time.has_supported_time_field = false;
  no_point_time.inspection_reason = "no_time_field_candidate";
  input.lidar_point_time = no_point_time;

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kOk);
  EXPECT_TRUE(snapshot.issues.empty());
}

TEST(TemporalDiagnosticsBuilderTest, LowConfidenceScanWindowProducesWarning) {
  auto input = BaseOkInput();
  input.lidar_scan_window = LowConfidenceFallbackWindow();

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kWarning);
  EXPECT_TRUE(HasIssueWithTitle(snapshot, "LiDAR scan window has low confidence"));
  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kLidarScanWindowLowConfidence));
}

TEST(TemporalDiagnosticsBuilderTest, DegradedImuCoverageProducesDegraded) {
  auto input = BaseOkInput();
  input.imu_coverage = DegradedCoverage();

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kDegraded);

  EXPECT_TRUE(HasIssueWithTitle(snapshot, "IMU does not properly cover the LiDAR scan window"));
  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kImuWindowIncomplete));
}

}  // namespace

TEST(TemporalDiagnosticsBuilderTest, OkTransformDoesNotCreateIssue) {
  auto input = BaseOkInput();
  input.transform_ages = {OkTransformAge()};

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kOk);
  EXPECT_FALSE(HasIssueWithReason(snapshot, TemporalFaultReason::kTfLookupFailed));
  EXPECT_FALSE(HasIssueWithReason(snapshot, TemporalFaultReason::kTfAgeTooHigh));
}

TEST(TemporalDiagnosticsBuilderTest, FailedTransformLookupProducesInvalid) {
  auto input = BaseOkInput();
  input.transform_ages = {FailedTransformLookup()};

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kInvalid);

  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kTfLookupFailed));
}

TEST(TemporalDiagnosticsBuilderTest, ExtrapolatedTransformProducesDegraded) {
  auto input = BaseOkInput();
  input.transform_ages = {ExtrapolatedTransformLookup()};

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kTfExtrapolationRequired));
}

TEST(TemporalDiagnosticsBuilderTest, StaleTransformProducesDegraded) {
  auto input = BaseOkInput();
  input.transform_ages = {StaleTransformLookup()};

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kTfAgeTooHigh));
}

TEST(TemporalDiagnosticsBuilderTest, FutureTransformProducesDegraded) {
  auto input = BaseOkInput();
  input.transform_ages = {FutureTransformLookup()};

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kDegraded);
  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kTfTransformFromFuture));
}

TEST(TemporalDiagnosticsBuilderTest, MissingLidarScanProducesInvalid) {
  model::TemporalObservation input;
  input.streams = {
      telemetry::MakeStreamTimingDiagnostic(telemetry::TemporalStreamId::kImu, OkTiming()),
      telemetry::MakeStreamTimingDiagnostic(telemetry::TemporalStreamId::kLidar, OkTiming()),
  };
  input.imu_buffer_size = 0;

  const TemporalDiagnosticsBuilder builder;
  const auto snapshot = builder.Build(input);

  EXPECT_EQ(snapshot.overall_status, telemetry::TemporalHealthStatus::kInvalid);
  EXPECT_TRUE(HasIssueWithReason(snapshot, TemporalFaultReason::kNoLidarScanReceivedYet));
}
}  // namespace causal_slam::diagnostics