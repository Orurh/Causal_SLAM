#pragma once

#include <string>

#include "diagnostics/temporal_diagnostics.h"

namespace causal_slam::render {

[[nodiscard]] std::string RenderMapUpdateDecisionJson(
    const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot);

}  // namespace causal_slam::render
