#pragma once

#include <string>

#include "application/offline_analysis/offline_temporal_report.h"

namespace causal_slam::render {

class OfflineTemporalReportHtmlRenderer final {
 public:
  [[nodiscard]] std::string Render(const causal_slam::offline_analysis::OfflineTemporalReport& report) const;
};

}  // namespace causal_slam::render