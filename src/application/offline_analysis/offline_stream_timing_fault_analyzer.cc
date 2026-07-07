#include "application/offline_analysis/offline_stream_timing_fault_analyzer.h"

#include <string>

namespace causal_slam::offline_analysis {
namespace {

void AddStreamTimingFault(StreamTimingFaultReport& report, const std::string& reason) {
  ++report.fault_reasons[reason];
}

}  // namespace

StreamTimingFaultReport BuildStreamTimingFaultReport(const OfflineTemporalReport& report) {
  StreamTimingFaultReport faults;

  const auto& lidar_timing = report.lidar_timing;
  if (!lidar_timing.has_period || lidar_timing.period_count == 0 || lidar_timing.period_mean_ms <= 0.0) {
    return faults;
  }

  if (lidar_timing.period_stddev_ms >= faults.lidar_period_jitter_threshold_ms) {
    faults.lidar_stream_timing_jitter_high = true;
    AddStreamTimingFault(faults, "lidar_stream_timing_jitter_high");
  }

  if (lidar_timing.period_min_ms < faults.lidar_period_short_threshold_ratio * lidar_timing.period_mean_ms) {
    faults.lidar_stream_timing_short_period = true;
    AddStreamTimingFault(faults, "lidar_stream_timing_short_period");
  }

  if (lidar_timing.period_max_ms > faults.lidar_period_long_threshold_ratio * lidar_timing.period_mean_ms) {
    faults.lidar_stream_timing_long_period = true;
    AddStreamTimingFault(faults, "lidar_stream_timing_long_period");
  }

  return faults;
}

}  // namespace causal_slam::offline_analysis
