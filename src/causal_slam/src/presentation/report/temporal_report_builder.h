#pragma once

#include "diagnostics/temporal_diagnostics.h"
#include "presentation/report/report_document.h"
#include "statistics/temporal_statistics.h"

namespace causal_slam::report {

class TemporalReportBuilder final {
 public:
  [[nodiscard]] ReportDocument BuildDiagnosticsReport(
      const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot)
      const;

  [[nodiscard]] ReportDocument BuildStatisticsReport(
      const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot)
      const;
};

}  // namespace causal_slam::report
