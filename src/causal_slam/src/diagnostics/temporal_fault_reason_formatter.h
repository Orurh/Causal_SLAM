#pragma once

#include <string>
#include <vector>

#include "diagnostics/temporal_diagnostics.h"

namespace causal_slam::diagnostics {

[[nodiscard]] std::string JoinFaultReasons(
    const std::vector<TemporalDiagnosticIssue>& issues);

}  // namespace causal_slam::diagnostics
