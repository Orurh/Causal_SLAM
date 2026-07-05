#pragma once

#include <string>

#include "application/offline_analysis/offline_temporal_report.h"

namespace causal_slam::render {

struct OfflineTemporalReportConsoleRenderContext {
  std::string bag_path;
  std::string report_path;
  std::string lidar_topic;
  std::string imu_topic;
};

class OfflineTemporalReportConsoleRenderer final {
 public:
  [[nodiscard]] std::string Render(const OfflineTemporalReportConsoleRenderContext& context,
                                   const causal_slam::offline_analysis::OfflineTemporalReport& summary) const;
};

}  // namespace causal_slam::render
