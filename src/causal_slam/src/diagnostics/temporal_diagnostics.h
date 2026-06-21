#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "telemetry/temporal_health.h"
#include "model/temporal_observation.h"

namespace causal_slam::diagnostics {

enum class TemporalDiagnosticSeverity : std::uint8_t {
  kInfo,
  kWarning,
  kDegraded,
};

[[nodiscard]] const char* ToString(TemporalDiagnosticSeverity severity);

struct TemporalDiagnosticIssue {
  TemporalDiagnosticSeverity severity{TemporalDiagnosticSeverity::kInfo};

  std::string title;
  std::string explanation;
  std::string evidence;
  std::string suggested_action;
};

struct TemporalDiagnosticSnapshot {
  causal_slam::telemetry::TemporalHealthStatus overall_status{
      causal_slam::telemetry::TemporalHealthStatus::kOk};

  causal_slam::model::TemporalObservation observation;
  std::vector<TemporalDiagnosticIssue> issues;
};

class TemporalDiagnosticsBuilder final {
 public:
  [[nodiscard]] TemporalDiagnosticSnapshot Build(
      const causal_slam::model::TemporalObservation& observation) const;
};

}  // namespace causal_slam::diagnostics
