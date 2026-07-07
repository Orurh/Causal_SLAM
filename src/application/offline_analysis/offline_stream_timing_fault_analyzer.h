#pragma once

#include "application/offline_analysis/offline_temporal_report.h"

namespace causal_slam::offline_analysis {

[[nodiscard]] StreamTimingFaultReport BuildStreamTimingFaultReport(const OfflineTemporalReport& report);

}  // namespace causal_slam::offline_analysis
