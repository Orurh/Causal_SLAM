#include "offline_temporal_report_html_renderer.h"

#include "presentation/render/report_document_html_renderer.h"
#include "presentation/report/offline_temporal_report_document_builder.h"

namespace causal_slam::render {

std::string OfflineTemporalReportHtmlRenderer::Render(const causal_slam::offline_analysis::OfflineTemporalReport& report) const {
  const causal_slam::report::OfflineTemporalReportDocumentBuilder document_builder;
  const ReportDocumentHtmlRenderer html_renderer;

  return html_renderer.RenderPage(document_builder.Build(report));
}

}  // namespace causal_slam::render