#pragma once

#include <string>

#include "domain/diagnostics/temporal_diagnostics.h"
#include "domain/policy/map_update_decision.h"

namespace causal_slam::render {

[[nodiscard]] std::string RenderMapUpdateDecisionJson(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                                      const causal_slam::policy::MapUpdateDecision& map_update_decision);

}  // namespace causal_slam::render
