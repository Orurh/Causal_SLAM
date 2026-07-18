#include "offline_temporal_report_artifact_writer.h"

#include <fstream>
#include <ostream>

#include "presentation/render/offline_temporal_report_html_renderer.h"
#include "presentation/render/offline_temporal_report_json_renderer.h"

namespace causal_slam::render {

bool OfflineTemporalReportArtifactWriter::Write(const OfflineTemporalReportArtifactPaths& paths,
                                                const OfflineTemporalReportArtifactContext& context,
                                                const causal_slam::offline_analysis::OfflineTemporalReport& report,
                                                std::ostream& err) const {
  if (paths.json_report_path.empty()) {
    err << "Missing JSON report path\n";
    return false;
  }

  if (!WriteJson(paths.json_report_path, context, report, err)) {
    return false;
  }

  if (!paths.html_report_path.empty() && !WriteHtml(paths.html_report_path, report, err)) {
    return false;
  }

  return true;
}

bool OfflineTemporalReportArtifactWriter::WriteJson(const std::string& path, const OfflineTemporalReportArtifactContext& context,
                                                    const causal_slam::offline_analysis::OfflineTemporalReport& report,
                                                    std::ostream& err) const {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to open report file for writing: " << path << "\n";
    return false;
  }

  const OfflineTemporalReportJsonRenderer renderer;
  OfflineTemporalReportRenderContext render_context;
  render_context.bag_path = context.bag_path;
  render_context.lidar_topic = context.lidar_topic;
  render_context.imu_topic = context.imu_topic;

  file << renderer.Render(render_context, report);
  return true;
}

bool OfflineTemporalReportArtifactWriter::WriteHtml(const std::string& path,
                                                    const causal_slam::offline_analysis::OfflineTemporalReport& report,
                                                    std::ostream& err) const {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to open HTML report file for writing: " << path << "\n";
    return false;
  }

  const OfflineTemporalReportHtmlRenderer renderer;
  file << renderer.Render(report);
  return true;
}

}  // namespace causal_slam::render