#include "temporal_monitor_runtime_defaults.h"

namespace causal_slam::config {

TemporalMonitorRuntimeDefaults MakeDefaultTemporalMonitorRuntimeDefaults() {
  TemporalMonitorRuntimeDefaults defaults;

  // Conservative online SLAM defaults. These are application presets, not
  // mathematical constants. Tune them per robot/sensor latency budget.
  defaults.summary_period_ms = 2000.0;
  defaults.expected_imu_period_ms = 5.0;  // 200 Hz IMU.
  defaults.imu_buffer_retention_ms = 5000.0;
  defaults.gate_min_total_imu_samples_before_forward = 5;
  defaults.gate_min_window_imu_samples_before_forward = 2;

  defaults.pipeline.imu_gap_threshold_ms = 100.0;
  defaults.pipeline.lidar_gap_threshold_ms = 500.0;

  defaults.pipeline.lidar_scan_window.fallback_scan_duration_ms = 100.0;  // 10 Hz LiDAR.
  defaults.pipeline.lidar_scan_window.min_measured_scan_duration_ms = 1.0;
  defaults.pipeline.lidar_scan_window.max_measured_scan_duration_ms = 500.0;
  defaults.pipeline.lidar_scan_window.prefer_measured_header_period = true;

  defaults.pipeline.transform_age.max_transform_age_ms = 50.0;
  defaults.pipeline.transform_age.max_future_tolerance_ms = 1.0;

  return defaults;
}

}  // namespace causal_slam::config
