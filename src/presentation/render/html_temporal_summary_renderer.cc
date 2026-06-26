#include "html_temporal_summary_renderer.h"

#include <sstream>
#include <string>
#include <string_view>

#include "presentation/report/report_document.h"
#include "presentation/report/temporal_report_builder.h"
#include "domain/policy/map_update_decision.h"

namespace causal_slam::render {
namespace {

std::string EscapeHtml(std::string_view input) {
  std::string output;
  output.reserve(input.size());

  for (const char ch : input) {
    switch (ch) {
      case '&':
        output += "&amp;";
        break;
      case '<':
        output += "&lt;";
        break;
      case '>':
        output += "&gt;";
        break;
      case '"':
        output += "&quot;";
        break;
      case '\'':
        output += "&#39;";
        break;
      default:
        output += ch;
        break;
    }
  }

  return output;
}

std::string CssClassForStatus(std::string_view status) {
  if (status == "OK") {
    return "status-ok";
  }

  if (status == "WARNING") {
    return "status-warning";
  }

  if (status == "DEGRADED") {
    return "status-degraded";
  }

  if (status == "INVALID") {
    return "status-invalid";
  }

  if (status == "ERROR") {
    return "status-invalid";
  }

  if (status == "INFO") {
    return "status-info";
  }

  return "status-neutral";
}

void RenderMetric(
    std::ostringstream& out,
    const causal_slam::report::ReportMetric& metric) {
  out << "<div class=\"metric\">"
      << "<span class=\"metric-name\">" << EscapeHtml(metric.name) << "</span>"
      << "<span class=\"metric-value\">" << EscapeHtml(metric.value) << "</span>"
      << "</div>\n";
}

void RenderRow(
    std::ostringstream& out,
    const causal_slam::report::ReportRow& row) {
  if (row.collapsed) {
    out << "<details class=\"row row-collapsed\">\n"
        << "  <summary class=\"row-head\">\n";
  } else {
    out << "<div class=\"row\">\n"
        << "  <div class=\"row-head\">\n";
  }

  out << "    <span class=\"row-label\">" << EscapeHtml(row.label)
      << "</span>\n";

  if (!row.status.empty()) {
    out << "    <span class=\"badge " << CssClassForStatus(row.status)
        << "\">" << EscapeHtml(row.status) << "</span>\n";
  }

  if (row.collapsed) {
    out << "  </summary>\n";
  } else {
    out << "  </div>\n";
  }

  if (!row.metrics.empty()) {
    out << "  <div class=\"metrics row-metrics\">\n";
    for (const auto& metric : row.metrics) {
      RenderMetric(out, metric);
    }
    out << "  </div>\n";
  }

  if (!row.reason.empty()) {
    out << "  <div class=\"row-line\"><span>reason</span><code>"
        << EscapeHtml(row.reason) << "</code></div>\n";
  }

  if (!row.detail.empty()) {
    out << "  <div class=\"row-line\"><span>detail</span><code>"
        << EscapeHtml(row.detail) << "</code></div>\n";
  }

  if (!row.explanation.empty()) {
    out << "  <p class=\"row-text\"><strong>Why:</strong> "
        << EscapeHtml(row.explanation) << "</p>\n";
  }

  if (!row.evidence.empty()) {
    out << "  <p class=\"row-text\"><strong>Evidence:</strong> "
        << EscapeHtml(row.evidence) << "</p>\n";
  }

  if (!row.suggested_action.empty()) {
    out << "  <p class=\"row-text\"><strong>Action:</strong> "
        << EscapeHtml(row.suggested_action) << "</p>\n";
  }

  if (row.collapsed) {
    out << "</details>\n";
  } else {
    out << "</div>\n";
  }
}

void RenderSection(
    std::ostringstream& out,
    const causal_slam::report::ReportSection& section) {
  out << "<section class=\"card\" id=\"" << EscapeHtml(section.id) << "\">\n"
      << "  <h2>" << EscapeHtml(section.title) << "</h2>\n";

  if (section.metrics.empty() && section.rows.empty()) {
    out << "  <p class=\"empty\">" << EscapeHtml(section.empty_message)
        << "</p>\n"
        << "</section>\n";
    return;
  }

  if (!section.metrics.empty()) {
    out << "  <div class=\"metrics\">\n";
    for (const auto& metric : section.metrics) {
      RenderMetric(out, metric);
    }
    out << "  </div>\n";
  }

  if (!section.rows.empty()) {
    out << "  <div class=\"rows\">\n";
    for (const auto& row : section.rows) {
      RenderRow(out, row);
    }
    out << "  </div>\n";
  }

  out << "</section>\n";
}

std::string RenderDocumentBody(
    const causal_slam::report::ReportDocument& document) {
  std::ostringstream out;

  if (!document.title.empty()) {
    out << "<h1>" << EscapeHtml(document.title) << "</h1>\n";
  }

  for (const auto& section : document.sections) {
    RenderSection(out, section);
  }

  return out.str();
}

const char* PageCss() {
  return R"CSS(
:root {
  color-scheme: light dark;
  font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
body {
  margin: 0;
  padding: 32px;
  background: #111827;
  color: #e5e7eb;
}
main {
  max-width: 1180px;
  margin: 0 auto;
}
h1 {
  margin: 0 0 20px;
  font-size: 28px;
}
h2 {
  margin: 0 0 14px;
  font-size: 18px;
}
.grid {
  display: grid;
  gap: 16px;
}
.card {
  border: 1px solid #374151;
  border-radius: 14px;
  background: #1f2937;
  padding: 18px;
}
.metrics {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
}
.metric {
  border: 1px solid #4b5563;
  border-radius: 10px;
  padding: 8px 10px;
  background: #111827;
}
.metric-name {
  display: block;
  color: #9ca3af;
  font-size: 12px;
}
.metric-value {
  display: block;
  margin-top: 2px;
  font-weight: 650;
}
.rows {
  display: grid;
  gap: 10px;
  margin-top: 10px;
}
.row {
  border: 1px solid #374151;
  border-radius: 12px;
  padding: 12px;
  background: #111827;
}
.row-head {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
}
.row-collapsed > summary {
  cursor: pointer;
  list-style: none;
}
.row-collapsed > summary::-webkit-details-marker {
  display: none;
}
.row-collapsed > summary::before {
  content: "▸";
  color: #9ca3af;
  margin-right: 8px;
}
.row-collapsed[open] > summary::before {
  content: "▾";
}
.row-label {
  font-weight: 650;
}
.row-metrics {
  margin-top: 10px;
}
.row-line {
  display: flex;
  gap: 10px;
  margin-top: 8px;
  color: #d1d5db;
}
.row-line span {
  min-width: 68px;
  color: #9ca3af;
}
.row-text {
  margin: 8px 0 0;
  color: #d1d5db;
}
.badge {
  border-radius: 999px;
  padding: 4px 9px;
  font-size: 12px;
  font-weight: 750;
}
.status-ok {
  background: #064e3b;
  color: #a7f3d0;
}
.status-warning {
  background: #78350f;
  color: #fde68a;
}
.status-degraded {
  background: #7c2d12;
  color: #fed7aa;
}
.status-invalid {
  background: #7f1d1d;
  color: #fecaca;
}
.status-info {
  background: #1e3a8a;
  color: #bfdbfe;
}
.status-neutral {
  background: #374151;
  color: #e5e7eb;
}
.empty {
  color: #9ca3af;
}
code {
  color: #bfdbfe;
  overflow-wrap: anywhere;
}
)CSS";
}

}  // namespace

std::string HtmlTemporalSummaryRenderer::RenderDiagnostics(
    const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
    const causal_slam::policy::MapUpdateDecision& map_update_decision) const {
  const causal_slam::report::TemporalReportBuilder builder;
  return RenderDocumentBody(
      builder.BuildDiagnosticsReport(snapshot, map_update_decision));
}

std::string HtmlTemporalSummaryRenderer::RenderStatistics(
    const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const {
  const causal_slam::report::TemporalReportBuilder builder;
  return RenderDocumentBody(builder.BuildStatisticsReport(snapshot));
}

std::string HtmlTemporalSummaryRenderer::RenderPage(
    const causal_slam::diagnostics::TemporalDiagnosticSnapshot& diagnostics,
    const causal_slam::policy::MapUpdateDecision& map_update_decision,
    const causal_slam::statistics::TemporalStatisticsSnapshot& statistics) const {
  std::ostringstream out;

  out << "<!doctype html>\n"
      << "<html lang=\"en\">\n"
      << "<head>\n"
      << "  <meta charset=\"utf-8\">\n"
      << "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
      << "  <title>Causal-SLAM Temporal Report</title>\n"
      << "  <style>\n" << PageCss() << "  </style>\n"
      << "</head>\n"
      << "<body>\n"
      << "<main>\n"
      << "<h1>Causal-SLAM Temporal Report</h1>\n"
      << "<div class=\"grid\">\n"
      << RenderDiagnostics(diagnostics, map_update_decision)
      << RenderStatistics(statistics)
      << "</div>\n"
      << "</main>\n"
      << "</body>\n"
      << "</html>\n";

  return out.str();
}

}  // namespace causal_slam::render
