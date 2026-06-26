# causal_slam_ros1

ROS1 Noetic adapter for the Causal-SLAM temporal integrity monitor.

This package is intentionally a thin ROS1 wrapper around the shared ROS-free
Causal-SLAM core.

## Purpose

`causal_slam_ros1` exists to run the same temporal integrity logic in ROS1
Noetic without duplicating the core C++ implementation.

The shared core is reused from the parent `causal_slam` repository:

~~~text
sensor_msgs/PointCloud2 + sensor_msgs/Imu
  -> ROS1 adapter conversion
  -> causal_slam::pipeline::TemporalMonitorPipeline
  -> std_msgs/Bool + std_msgs/String diagnostic topics
~~~

The ROS1 adapter should only:

- subscribe to ROS1 topics;
- convert ROS1 messages into internal Causal-SLAM inputs;
- call the shared temporal pipeline;
- publish diagnostic topics;
- print readable summaries.

## Published topics

~~~text
/causal_slam/map_update_allowed  std_msgs/Bool
/causal_slam/temporal_health     std_msgs/String
/causal_slam/map_update_reason   std_msgs/String
/causal_slam/fault_reasons       std_msgs/String
~~~

Healthy example:

~~~text
map_update_allowed: true
temporal_health: OK
map_update_reason: temporal_health_ok
fault_reasons: none
~~~

Degraded example:

~~~text
map_update_allowed: false
temporal_health: DEGRADED
map_update_reason: temporal_health_degraded
fault_reasons: imu_window_incomplete
~~~

## Recommended Noetic workspace layout

Do not put the whole `causal_slam` ROS2 repository directly into
`catkin_ws/src`.

The repository root contains a ROS2 `package.xml` with `ament_cmake`.
A ROS1 catkin workspace may try to process it as a catkin package and fail.

Recommended layout:

~~~text
~/projects/
└── causal_slam/                         # full repository

~/catkin_ws/
└── src/
    └── causal_slam_ros1 -> ~/projects/causal_slam/ros1/causal_slam_ros1
~~~

Create the symlink:

~~~bash
mkdir -p ~/catkin_ws/src
ln -sfn /absolute/path/to/causal_slam/ros1/causal_slam_ros1 \
  ~/catkin_ws/src/causal_slam_ros1
~~~

## Build in ROS1 Noetic

From the ROS1 Noetic catkin workspace:

~~~bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash

catkin_make \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCAUSAL_SLAM_ROOT=/absolute/path/to/causal_slam

source devel/setup.bash
~~~

`CAUSAL_SLAM_ROOT` must point to the full Causal-SLAM repository root, the
directory that contains the shared `src/` folder.

## Run

Recommended launch-file run:

~~~bash
roslaunch causal_slam_ros1 temporal_monitor.launch
~~~

Run with a custom config:

~~~bash
roslaunch causal_slam_ros1 temporal_monitor.launch \
  config:=/absolute/path/to/temporal_monitor.yaml
~~~

Direct `rosrun` is also possible:

~~~bash
rosrun causal_slam_ros1 causal_slam_ros1_temporal_monitor_node \
  _imu_topic:=/imu/data \
  _lidar_topic:=/points \
  _expected_imu_period_ms:=20.0
~~~

Useful parameters:

~~~text
_imu_topic:=/imu/data
_lidar_topic:=/points
_summary_period_ms:=2000.0
_expected_imu_period_ms:=20.0
_imu_gap_threshold_ms:=100.0
_lidar_gap_threshold_ms:=500.0
_lidar_scan_duration_ms:=100.0
_lidar_stamp_policy:=scan_end
_imu_buffer_retention_ms:=5000.0
_max_missing_prefix_ms:=40.0
_max_missing_suffix_ms:=40.0
_max_internal_gap_ms:=100.0
~~~

## Design boundaries

Shared ROS-free logic lives in:

~~~text
src/application/temporal_monitor/
src/domain/sensors/pointcloud/
src/domain/sensors/imu/
src/domain/diagnostics/
src/domain/sensors/lidar/
src/domain/model/
src/domain/policy/
src/domain/statistics/
src/domain/telemetry/
src/presentation/render/
~~~

ROS1-specific code lives in:

~~~text
ros1/causal_slam_ros1/
~~~

ROS2-specific code lives in:

~~~text
src/apps/ros2/
src/adapters/ros2/
~~~

Dependency direction:

~~~text
ROS1 adapter
  -> shared ROS-free pipeline/core

ROS2 adapter
  -> shared ROS-free pipeline/core

shared core
  -> no ROS dependency
~~~

## Current status

This ROS1 adapter is an initial skeleton.

The ROS2 Jazzy package and smoke test are currently validated.

The ROS1 adapter still needs to be compiled and tested in a real Noetic
workspace with real or simulated `/points` and `/imu/data` topics.

## Cross-ROS development contract

See also:

~~~text
docs/ROS_COMPATIBILITY.md
~~~

The ROS1 adapter must stay thin and must not duplicate temporal integrity logic
from the shared core.
