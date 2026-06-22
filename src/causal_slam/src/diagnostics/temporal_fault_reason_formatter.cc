#include "diagnostics/temporal_fault_reason_formatter.h"

#include <cstddef>
#include <sstream>

namespace causal_slam::diagnostics {

std::string JoinFaultReasons(
    const std::vector<TemporalDiagnosticIssue>& issues) {
  if (issues.empty()) {
    return "none";
  }

  std::ostringstream out;

  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (i > 0) {
      out << ",";
    }

    out << ToString(issues[i].reason);
  }

  return out.str();
}

}  // namespace causal_slam::diagnostics
