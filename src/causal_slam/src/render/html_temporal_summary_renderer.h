#pragma once

#include <string>

#include "diagnostics/temporal_diagnostics.h"
#include "statistics/temporal_statistics.h"

namespace causal_slam::render {

class HtmlTemporalSummaryRenderer final {
 public:
  [[nodiscard]] std::string RenderPage(
      const causal_slam::diagnostics::TemporalDiagnosticSnapshot& diagnostics,
      const causal_slam::statistics::TemporalStatisticsSnapshot& statistics)
      const;

  [[nodiscard]] std::string RenderDiagnostics(
      const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) const;

  [[nodiscard]] std::string RenderStatistics(
      const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const;
};

}  // namespace causal_slam::render
