#pragma once

#include <string>

#include "diagnostics/temporal_diagnostics.h"
#include "statistics/temporal_statistics.h"

namespace causal_slam::render {

class ConsoleTemporalSummaryRenderer final {
 public:
  [[nodiscard]] std::string Render(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) const;

  [[nodiscard]] std::string RenderStatistics(const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const;
};

}  // namespace causal_slam::render