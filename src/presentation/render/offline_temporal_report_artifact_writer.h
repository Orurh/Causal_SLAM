#pragma once

#include <iosfwd>
#include <string>

#include "application/offline_analysis/offline_temporal_report.h"

namespace causal_slam::render {

struct OfflineTemporalReportArtifactPaths {
  std::string json_report_path;
  std::string html_report_path;
};

struct OfflineTemporalReportArtifactContext {
  std::string bag_path;
  std::string lidar_topic;
  std::string imu_topic;
};

class OfflineTemporalReportArtifactWriter final {
 public:
  [[nodiscard]] bool Write(const OfflineTemporalReportArtifactPaths& paths, const OfflineTemporalReportArtifactContext& context,
                           const causal_slam::offline_analysis::OfflineTemporalReport& report, std::ostream& err) const;

 private:
  [[nodiscard]] bool WriteJson(const std::string& path, const OfflineTemporalReportArtifactContext& context,
                               const causal_slam::offline_analysis::OfflineTemporalReport& report, std::ostream& err) const;

  [[nodiscard]] bool WriteHtml(const std::string& path, const causal_slam::offline_analysis::OfflineTemporalReport& report,
                               std::ostream& err) const;
};

}  // namespace causal_slam::render