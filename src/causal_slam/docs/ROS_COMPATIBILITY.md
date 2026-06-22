# ROS compatibility contract

Causal-SLAM must support multiple ROS runtimes through thin adapter layers.

Current targets:

- ROS2 Jazzy
- ROS1 Noetic

The core temporal integrity logic must stay independent from ROS.

## Architecture rule

Allowed direction:

~~~text
ROS2 node/adapters
  -> shared ROS-free core/pipeline

ROS1 node/adapters
  -> shared ROS-free core/pipeline

shared core/pipeline
  -> no ROS dependency
~~~

Forbidden direction:

~~~text
shared core/pipeline
  -> ROS2 headers
  -> ROS1 headers
  -> rclcpp
  -> roscpp
  -> sensor_msgs directly
  -> std_msgs directly
  -> tf2_ros directly
~~~

## Shared ROS-free modules

These modules must stay reusable by both ROS1 and ROS2:

~~~text
src/core/
src/coverage/
src/diagnostics/
src/lidar/
src/model/
src/pipeline/
src/pointcloud/
src/policy/
src/render/
src/statistics/
src/telemetry/
~~~

They may use standard C++ and internal Causal-SLAM types.

They must not include:

~~~text
rclcpp/
ros/
sensor_msgs/
std_msgs/
builtin_interfaces/
tf2_ros/
nav_msgs/
geometry_msgs/
~~~

## ROS2-specific modules

ROS2-specific code belongs here:

~~~text
src/nodes/
src/ros_adapters/
~~~

ROS2 code may include:

~~~text
rclcpp/
sensor_msgs/msg/
std_msgs/msg/
builtin_interfaces/msg/
~~~

ROS2 code should only:

- subscribe to ROS2 topics;
- convert ROS2 messages into internal inputs;
- call shared pipeline/core;
- publish ROS2 diagnostics;
- log readable summaries.

## ROS1-specific modules

ROS1-specific code belongs here:

~~~text
ros1/causal_slam_ros1/
~~~

ROS1 code may include:

~~~text
ros/
sensor_msgs/
std_msgs/
~~~

ROS1 code should only:

- subscribe to ROS1 topics;
- convert ROS1 messages into internal inputs;
- call shared pipeline/core;
- publish ROS1 diagnostics;
- log readable summaries.

## Message conversion rule

ROS message types must not leak into shared core.

Correct:

~~~text
sensor_msgs::msg::PointCloud2
  -> ros_adapters::ToPointCloud2CloudView(...)
  -> pointcloud::PointCloud2CloudView
  -> pipeline/core
~~~

Correct:

~~~text
sensor_msgs::PointCloud2
  -> causal_slam_ros1::ToPointCloud2CloudView(...)
  -> pointcloud::PointCloud2CloudView
  -> pipeline/core
~~~

Wrong:

~~~text
pipeline::TemporalMonitorPipeline
  -> sensor_msgs::msg::PointCloud2
~~~

Wrong:

~~~text
pointcloud::PointCloud2TimeFieldExtractor
  -> sensor_msgs::PointCloud2
~~~

## Feature development rule

When adding a new analyzer, policy, statistic, or diagnostic reason:

1. Add ROS-free model/core/pipeline code first.
2. Add pure C++ tests first.
3. Add ROS2 adapter/node wiring after the core test passes.
4. Add ROS1 adapter/node wiring only as conversion and publication code.
5. Extend architecture/audit scripts if a new module boundary is intentional.

## C++ compatibility rule

Shared core should avoid unnecessary compiler-specific or ROS-version-specific features.

Current preferred baseline for shared core:

~~~text
C++20-compatible where practical.
No ROS dependency.
No mutable global state.
No singleton-based ownership.
Structured diagnostics instead of log-only state.
~~~

C++23 may be used in the ROS2 Jazzy package only when it does not leak into the shared core needed by ROS1 Noetic.

## Required checks

Before commit:

~~~bash
./scripts/check_arch_deps.py
./scripts/audit_ros1_adapter.py
./scripts/audit_ros1_compatibility.py
git diff --check
~~~

Full ROS2 validation:

~~~bash
cd /home/orurh/Документы/VSCode/causal_slam_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select causal_slam
colcon test --packages-select causal_slam --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
src/causal_slam/scripts/smoke_temporal_gate.sh
~~~

ROS1 validation must be done in a real Noetic catkin workspace.
