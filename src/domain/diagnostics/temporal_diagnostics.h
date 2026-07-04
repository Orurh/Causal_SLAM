#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "domain/model/temporal_observation.h"
#include "domain/telemetry/temporal_health.h"

namespace causal_slam::diagnostics {

enum class TemporalDiagnosticSeverity : std::uint8_t {
  kInfo,
  kWarning,
  kDegraded,
  kInvalid,
};

enum class TemporalFaultReason : std::uint8_t {
  kNone,

  kStreamTimingUnstable,
  kImuStreamTimingUnstable,
  kLidarStreamTimingUnstable,

  kNoLidarScanReceivedYet,
  kNoImuSampleReceivedYet,
  kLidarStreamStale,
  kImuStreamStale,

  kImuWindowIncomplete,

  kLidarPointTimeUnsupported,
  kLidarPointTimeExtractionFailed,
  kLidarScanWindowLowConfidence,

  kTfLookupFailed,
  kTfExtrapolationRequired,
  kTfAgeTooHigh,
  kTfTransformFromFuture,
};

[[nodiscard]] const char* ToString(TemporalDiagnosticSeverity severity);
[[nodiscard]] const char* ToString(TemporalFaultReason reason);

struct TemporalDiagnosticIssue {
  TemporalDiagnosticSeverity severity{TemporalDiagnosticSeverity::kInfo};
  TemporalFaultReason reason{TemporalFaultReason::kNone};

  std::string title;
  std::string explanation;
  std::string evidence;
  std::string suggested_action;
};

struct TemporalDiagnosticSnapshot {
  causal_slam::telemetry::TemporalHealthStatus overall_status{causal_slam::telemetry::TemporalHealthStatus::kOk};

  // causal_slam::policy::MapUpdateDecision map_update_decision;
  causal_slam::model::TemporalObservation observation;
  std::vector<TemporalDiagnosticIssue> issues;
};

class TemporalDiagnosticsBuilder final {
 public:
  [[nodiscard]] TemporalDiagnosticSnapshot Build(const causal_slam::model::TemporalObservation& observation) const;
};

}  // namespace causal_slam::diagnostics
