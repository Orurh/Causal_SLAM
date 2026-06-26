#include "map_update_decision_json_renderer.h"

#include <cstddef>
#include <sstream>
#include <string>
#include <string_view>

#include "domain/policy/map_update_decision.h"

namespace causal_slam::render {
namespace {

std::string JsonEscape(std::string_view value) {
  std::string escaped;
  escaped.reserve(value.size());

  for (const char ch : value) {
    switch (ch) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += ch;
        break;
    }
  }

  return escaped;
}

void AppendJsonString(std::ostringstream& out, std::string_view value) {
  out << '"' << JsonEscape(value) << '"';
}

}  // namespace

std::string RenderMapUpdateDecisionJson(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                        const causal_slam::policy::MapUpdateDecision& map_update_decision) {
  std::ostringstream out;

  out << "{";
  out << "\"has_lidar_scan\":" << (snapshot.observation.has_lidar_scan ? "true" : "false") << ",";
  out << "\"scan_stamp_ns\":" << snapshot.observation.latest_lidar_header_stamp_ns << ",";
  out << "\"frame_id\":";
  AppendJsonString(out, snapshot.observation.latest_lidar_frame_id);
  out << ",";
  out << "\"allowed\":" << (map_update_decision.map_update_allowed ? "true" : "false") << ",";
  out << "\"health\":";
  AppendJsonString(out, causal_slam::telemetry::ToString(snapshot.overall_status));
  out << ",";
  out << "\"reason\":";
  AppendJsonString(out, causal_slam::policy::ToString(map_update_decision.reason));
  out << ",";
  out << "\"fault_reasons\":[";

  for (std::size_t i = 0; i < snapshot.issues.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    AppendJsonString(out, causal_slam::diagnostics::ToString(snapshot.issues[i].reason));
  }

  out << "],";
  out << "\"evidence\":[";

  for (std::size_t i = 0; i < snapshot.issues.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    AppendJsonString(out, snapshot.issues[i].evidence);
  }

  out << "]";
  out << "}";

  return out.str();
}

}  // namespace causal_slam::render
