# Causal-SLAM

[Русская версия](README.ru.md)

**Causal-SLAM** is an experimental temporal integrity layer for LiDAR/IMU SLAM/LIO pipelines, built with ROS 2 and C++17.

> Before trusting geometry, verify that the measurements were temporally compatible.

The project does not build a map and does not replace FAST-LIO, LIO-SAM, or another SLAM/LIO estimator. It analyzes input data before fusion and produces diagnostics, health status, and forwarding/map-update decisions.

## Why

A LiDAR scan is acquired over a time interval, while the IMU measures sensor motion throughout that interval. The presence of both topics and monotonic timestamps does not guarantee that a particular scan is correctly covered by IMU data.

Missing samples, message reordering, incorrect point-time interpretation, or unavailable TF data may later appear as registration failures, drift, or unstable map updates.

Causal-SLAM asks an earlier question:

**were these measurements temporally valid to fuse?**

## Architecture

```text
LiDAR / IMU / TF / rosbag2
            |
            v
     Causal-SLAM
  temporal integrity layer
            |
            +--> diagnostics / health / evidence
            +--> map update decision
            +--> checked LiDAR
            |
            v
      SLAM / LIO estimator
```

The domain/application core does not depend on `rclcpp`. The ROS 2 layer handles messages, QoS, TF, parameters, rosbag2, and data adaptation.

## Current capabilities

Causal-SLAM checks:

- LiDAR and IMU stream stability;
- gaps, jitter, and message reordering;
- the LiDAR scan time window;
- `PointCloud2` time-field availability and interpretation;
- Ouster-style `t` and `offset_time` `UINT32` fields;
- IMU coverage over the LiDAR scan window;
- missing edge coverage and internal IMU gaps;
- TF availability, age, and future timestamps;
- whether LiDAR should be forwarded and map update should be allowed.

Available components include:

- a live ROS 2 monitor;
- a checked LiDAR topic;
- structured diagnostics;
- a JSON decision topic;
- offline rosbag2 analysis;
- console summaries;
- JSON reports;
- optional HTML rendering.

## Limitations

Causal-SLAM currently does not:

- correct timestamps;
- perform deskew;
- estimate calibration or IMU bias;
- build maps or trajectories;
- replace a SLAM/LIO estimator;
- guarantee improved maps on arbitrary data.

The current goal is to detect and explain temporal faults first. Controlled fault injection and downstream SLAM before/after comparisons are the next validation stage.

## Requirements

- Ubuntu with ROS 2 Jazzy;
- C++17;
- CMake 3.20+;
- `colcon`;
- rosbag2 for offline analysis.

The runtime is currently implemented for ROS 2. Keeping the core C++17-compatible leaves room for a separate ROS 1 Noetic adapter later, but a ROS 1 runtime is not currently claimed as ready.

## Build

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/causal_slam_ws/src
cd ~/causal_slam_ws/src

git clone https://github.com/Orurh/Causal_SLAM.git causal_slam

cd ~/causal_slam_ws
colcon build \
  --packages-select causal_slam \
  --symlink-install

source install/setup.bash
```

The repository Makefile also provides:

```bash
make build
make test
make check
```

## Online monitor

```bash
source /opt/ros/jazzy/setup.bash
source ~/causal_slam_ws/install/setup.bash

ros2 run causal_slam temporal_monitor_node --ros-args \
  --params-file ~/causal_slam_ws/src/causal_slam/config/temporal_gate.yaml
```

Main configuration:

```text
config/temporal_gate.yaml
```

It defines input and output topics, QoS, gate mode, IMU coverage thresholds, LiDAR scan-window parameters, and TF monitoring.

## Main outputs

```text
/causal_slam/map_update_allowed
/causal_slam/temporal_health
/causal_slam/map_update_reason
/causal_slam/fault_reasons
/causal_slam/map_update_decision_json
/causal_slam/checked_lidar
```

For actual gating, the downstream SLAM pipeline must consume:

```text
/causal_slam/checked_lidar
```

instead of the raw LiDAR topic.

## Gate modes

```text
observe
  diagnostics only; LiDAR is forwarded

drop_invalid
  INVALID states are blocked

drop_degraded
  INVALID states and DEGRADED states with a hard fusion blocker
  are blocked; diagnostic-only DEGRADED states may still pass

strict
  only fully healthy data is forwarded after the minimum required
  timing evidence has been collected
```

## Offline rosbag2 analysis

Offline mode provides reproducible dataset analysis without live-playback timing artifacts.

```bash
ros2 run causal_slam causal_slam_analyze_bag \
  --bag /path/to/rosbag2 \
  --lidar-topic /ouster/points \
  --imu-topic /ouster/imu \
  --report /tmp/causal_slam_report.json \
  --html-report /tmp/causal_slam_report.html
```

JSON is the primary machine-readable artifact. HTML is an optional report representation.

Important JSON sections:

```text
verdict
point_cloud2_capability
lidar_scan_windows
imu_coverage
stream_timing_faults
```

## Result interpretation

```text
OK
  temporal checks passed

WARNING
  suspicious but non-critical evidence was detected

DEGRADED
  faults or insufficient confidence were detected;
  the action depends on the reason and gate policy

INVALID
  data is insufficient or the state is invalid
```

Example fault reasons:

```text
lidar_stream_timing_jitter_high
lidar_stream_timing_short_period
lidar_stream_timing_long_period
imu_stream_timing_jitter_high
imu_stream_timing_jitter_suspicious
imu_window_incomplete
lidar_point_time_unsupported
lidar_point_time_extraction_failed
lidar_scan_window_low_confidence
tf_lookup_failed
tf_extrapolation_required
tf_age_too_high
tf_transform_from_future
```

## Current validation

The project is being evaluated on public LiDAR/IMU datasets. Healthy datasets should be recognized as temporally compatible rather than forced into a fault classification. Incomplete or ambiguous datasets should produce an explicit reason and confidence level.

Next validation steps:

1. remove selected IMU messages;
2. inject a time offset;
3. reorder messages;
4. compare diagnostics and downstream SLAM behavior;
5. separate temporal faults from geometry, calibration, and estimator issues.

## Status

**Experimental MVP.**

The project is currently useful as a temporal diagnostics/gating layer and a reproducible offline evidence harness. It is not production safety software.

Project overview in Russian:  
[“Were LiDAR and IMU temporally valid to fuse?” — Setka](https://setka.ru/posts/019f770c-7f4c-71b3-a53f-e7ac6af67b33)