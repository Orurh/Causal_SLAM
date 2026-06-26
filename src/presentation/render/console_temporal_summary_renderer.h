#pragma once

#include <string>

#include "domain/diagnostics/temporal_diagnostics.h"
#include "domain/policy/map_update_decision.h"
#include "domain/statistics/temporal_statistics.h"

namespace causal_slam::render {

class ConsoleTemporalSummaryRenderer final {
 public:
  [[nodiscard]] std::string Render(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                   const causal_slam::policy::MapUpdateDecision& map_update_decision) const;

  [[nodiscard]] std::string RenderStatistics(const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const;
};

}  // namespace causal_slam::render