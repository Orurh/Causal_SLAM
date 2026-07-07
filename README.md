# Causal-SLAM

Causal-SLAM is a ROS 2 / C++ temporal integrity layer for LiDAR/IMU SLAM/LIO pipelines.

Core idea: **before trusting SLAM geometry, verify that sensor measurements were temporally valid to fuse.**

It is not a new SLAM estimator, not a FAST-LIO/LIO-SAM replacement, and not a GUI. It is a temporal monitor / guard in front of SLAM.

## What it does now

Causal-SLAM checks:

- LiDAR/IMU stream timing stability;
- IMU coverage over the LiDAR scan window;
- `PointCloud2` time-field availability and safety;
- TF lookup / stale / future transforms;
- whether map update should be allowed.

Main output topics:

```text
/causal_slam/map_update_allowed
/causal_slam/temporal_health
/causal_slam/map_update_reason
/causal_slam/fault_reasons
/causal_slam/map_update_decision_json
/causal_slam/checked_lidar
```

## What it does not do yet

Causal-SLAM does not yet:

- correct timestamps;
- perform deskew correction;
- build maps;
- replace a SLAM estimator;
- guarantee visual improvement unless the downstream SLAM pipeline consumes `/causal_slam/checked_lidar`.

## Build

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/causal_slam_ws/src
cd ~/causal_slam_ws/src
git clone <repo-url> causal_slam

cd ~/causal_slam_ws
colcon build --packages-select causal_slam
source install/setup.bash
```

Or from the repository:

```bash
make build
```

## Online run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run causal_slam temporal_monitor_node --ros-args \
  --params-file src/causal_slam/config/temporal_gate.yaml
```

From the repository root:

```bash
ros2 run causal_slam temporal_monitor_node --ros-args \
  --params-file config/temporal_gate.yaml
```

Main config:

```text
config/temporal_gate.yaml
```

It controls input topics, output topics, gate mode, IMU thresholds, LiDAR scan window settings, and TF monitoring.

## Gate modes

```text
observe        diagnostics only
drop_invalid   block invalid states
drop_degraded  block degraded and invalid states
strict         forward only fully healthy data
```

To use the gate with SLAM, feed the downstream SLAM pipeline from:

```text
/causal_slam/checked_lidar
```

instead of the raw LiDAR topic.

## Offline rosbag analysis

Offline mode is a reproducible evidence harness for datasets.

```bash
ros2 run causal_slam causal_slam_analyze_bag \
  --bag /path/to/rosbag2 \
  --lidar-topic /ouster/points \
  --imu-topic /ouster/imu \
  --report /tmp/causal_slam_report.json \
  --html-report /tmp/causal_slam_report.html
```

Expected output:

- console summary;
- JSON report;
- optional HTML report.

Important JSON fields:

```text
verdict.health
verdict.reason
point_cloud2_capability
lidar_scan_windows
imu_coverage
stream_timing_faults
```

## How to read the result

Health:

```text
OK        temporal evidence is healthy
WARNING   suspicious but not critical
DEGRADED  unsafe for map update
INVALID   insufficient data or invalid state
```

Map update:

```text
true   acceptable for map update
false  temporal evidence is degraded/invalid
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

## Planned visual validation

The project needs a dataset where temporal faults visibly affect downstream SLAM/LIO.

Validation plan:

```text
1. Run SLAM on raw LiDAR/IMU.
2. Run the same SLAM with Causal-SLAM in front.
3. Feed SLAM from /causal_slam/checked_lidar instead of raw LiDAR.
4. Compare map/trajectory before and after.
5. Show a side-by-side GIF.
```

Expected useful case:

```text
raw SLAM:
  map smearing, ghosting, or corrupted map updates

SLAM through Causal-SLAM:
  degraded/invalid temporal windows are not used for map update
  map is visually more stable
```

## Status

Experimental MVP.

The project is currently useful as a temporal diagnostics/gating layer and offline evidence harness. It is not production safety software yet.
