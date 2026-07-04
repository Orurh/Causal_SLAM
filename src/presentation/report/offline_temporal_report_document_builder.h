#pragma once

#include "application/offline_analysis/offline_temporal_report.h"
#include "presentation/report/report_document.h"

namespace causal_slam::report {

class OfflineTemporalReportDocumentBuilder final {
 public:
  [[nodiscard]] ReportDocument Build(
      const causal_slam::offline_analysis::OfflineTemporalReport& report) const;
};

}  // namespace causal_slam::report
