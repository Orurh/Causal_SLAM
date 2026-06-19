#pragma once

#include <string>

#include "diagnostics/temporal_diagnostics.h"

namespace causal_slam::render {

class ConsoleTemporalSummaryRenderer final {
 public:
  [[nodiscard]] std::string Render(
      const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot) const;
};

}  // namespace causal_slam::render