#include "html_temporal_summary_renderer.h"

#include "presentation/render/report_document_html_renderer.h"
#include "presentation/report/temporal_report_builder.h"

namespace causal_slam::render {

std::string HtmlTemporalSummaryRenderer::RenderDiagnostics(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& snapshot,
                                                           const causal_slam::policy::MapUpdateDecision& map_update_decision) const {
  const causal_slam::report::TemporalReportBuilder builder;
  const ReportDocumentHtmlRenderer renderer;

  return renderer.RenderBody(builder.BuildDiagnosticsReport(snapshot, map_update_decision));
}

std::string HtmlTemporalSummaryRenderer::RenderStatistics(const causal_slam::statistics::TemporalStatisticsSnapshot& snapshot) const {
  const causal_slam::report::TemporalReportBuilder builder;
  const ReportDocumentHtmlRenderer renderer;

  return renderer.RenderBody(builder.BuildStatisticsReport(snapshot));
}

std::string HtmlTemporalSummaryRenderer::RenderPage(const causal_slam::diagnostics::TemporalDiagnosticSnapshot& diagnostics,
                                                    const causal_slam::policy::MapUpdateDecision& map_update_decision,
                                                    const causal_slam::statistics::TemporalStatisticsSnapshot& statistics) const {
  const causal_slam::report::TemporalReportBuilder builder;
  const auto diagnostics_document = builder.BuildDiagnosticsReport(diagnostics, map_update_decision);
  const auto statistics_document = builder.BuildStatisticsReport(statistics);

  const ReportDocumentHtmlRenderer renderer;
  return renderer.RenderPage("Causal-SLAM Temporal Report", {&diagnostics_document, &statistics_document});
}

}  // namespace causal_slam::render