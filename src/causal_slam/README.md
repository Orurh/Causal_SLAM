# Causal-SLAM

Causal-SLAM is a small ROS 2 / C++ tool for checking temporal integrity in LiDAR/IMU SLAM pipelines.

Idea: before trusting SLAM geometry, first check that sensor data is temporally valid.

This is not a new SLAM algorithm. It is a temporal monitor / guard that checks LiDAR, IMU and TF timing and reports whether map updates are safe.

## Supported version

Tested with:

- Ubuntu 24.04
- ROS 2 Jazzy
- C++20
- CMake 3.28+

ROS 1 support exists only as an experimental compatibility adapter.

## Build

    source /opt/ros/jazzy/setup.bash

    mkdir -p ~/causal_slam_ws/src
    cd ~/causal_slam_ws/src
    git clone <repo-url> causal_slam

    cd ~/causal_slam_ws
    colcon build --packages-select causal_slam
    source install/setup.bash

Or with Makefile:

    make build

## Run tests

    make test

Run smoke scenarios:

    make smoke

## Run

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

    ros2 run causal_slam temporal_monitor_node --ros-args \
      --params-file src/causal_slam/config/temporal_gate.yaml

If you run from inside the repository:

    ros2 run causal_slam temporal_monitor_node --ros-args \
      --params-file config/temporal_gate.yaml

## Configuration

Main config file:

    config/temporal_gate.yaml

Input and output topics, gate mode, IMU thresholds, LiDAR scan settings, TF monitoring and HTML report path are configured there.

## Main output topics

    /causal_slam/map_update_allowed
    /causal_slam/temporal_health
    /causal_slam/map_update_reason
    /causal_slam/fault_reasons
    /causal_slam/map_update_decision_json
    /causal_slam/checked_lidar

## Gate modes

    observe
    drop_invalid
    drop_degraded
    strict

Meaning:

- observe: only report diagnostics
- drop_invalid: block invalid temporal states
- drop_degraded: block degraded and invalid states
- strict: forward only fully healthy data

## What it checks

- LiDAR stream timing
- IMU stream timing
- IMU coverage over LiDAR scan window
- delayed or unstable timestamps
- PointCloud2 time-field diagnostics
- TF lookup failures
- stale or future TF transforms
- map update allow/block decision

## Output meaning

Health:

    OK
    WARNING
    DEGRADED
    INVALID

Map update:

    true  -> timing evidence is acceptable
    false -> timing evidence is degraded or invalid

Example fault reasons:

    stream_timing_unstable
    imu_window_incomplete
    lidar_point_time_unsupported
    lidar_point_time_extraction_failed
    lidar_scan_window_low_confidence
    tf_lookup_failed
    tf_extrapolation_required
    tf_age_too_high
    tf_transform_from_future

## Status

Experimental MVP.

Useful for testing temporal integrity around SLAM/LIO pipelines, but not a production safety system yet.

## License

MIT