#pragma once

#include "domain/diagnostics/temporal_diagnostics.h"
#include "domain/policy/map_update_decision.h"
#include "domain/statistics/temporal_statistics.h"
#include "presentation/report/report_document.h"

namespace causal_slam::report {

class TemporalReportBuilder final {
 public:
  [[nodiscard]] ReportDocument BuildDiagnosticsReport(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                                      const causal_slam::policy::MapUpdateDecision& map_update_decision) const;

  [[nodiscard]] ReportDocument BuildStatisticsReport(const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const;
};

}  // namespace causal_slam::report