#pragma once

#include <string>

#include "domain/diagnostics/temporal_diagnostics.h"
#include "domain/statistics/temporal_statistics.h"

namespace causal_slam::render {

class HtmlTemporalSummaryRenderer final {
 public:
  [[nodiscard]] std::string RenderPage(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& diagnostics,
                                       const causal_slam::policy::MapUpdateDecision& map_update_decision,
                                       const causal_slam::statistics::TemporalStatisticsSnapshot& statistics) const;

  [[nodiscard]] std::string RenderDiagnostics(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                              const causal_slam::policy::MapUpdateDecision& map_update_decision) const;

  [[nodiscard]] std::string RenderStatistics(const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const;
};

}  // namespace causal_slam::render
