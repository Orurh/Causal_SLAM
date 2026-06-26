# Causal-SLAM Architecture

Causal-SLAM is a Temporal Integrity Layer for SLAM/LIO pipelines.

Its main job is to validate whether LiDAR, IMU, TF, and related observations are
temporally compatible before geometry, scan matching, or map updates are trusted.

Tagline:

```text
Before trusting geometry, verify time.
```

## What this project is

Causal-SLAM is:

- a temporal integrity monitor for robotics perception pipelines;
- a ROS2/C++ guard around LiDAR/IMU/TF timing assumptions;
- a diagnostics and gating layer for map update decisions;
- a tool for detecting temporal faults before they corrupt downstream SLAM/LIO.

## What this project is not

Causal-SLAM is not:

- a SLAM estimator;
- a LiDAR odometry implementation;
- a loop-closure backend;
- a replacement for FAST-LIO, Faster-LIO, LIO-SAM, or other estimators;
- an offline calibration tool;
- a generic dashboard for topic rates.

## Logical dependency direction

The intended dependency direction is:

```text
apps -> adapters -> application -> domain
presentation -> domain/application result
platform is utility-only
```

Domain code must not depend on ROS.

## Layers

### Domain

Path:

```text
src/domain/
```

Contains pure C++ model and logic:

```text
src/domain/time/
src/domain/model/
src/domain/telemetry/
src/domain/sensors/imu/
src/domain/sensors/lidar/
src/domain/sensors/pointcloud/
src/domain/sensors/transform/
src/domain/diagnostics/
src/domain/policy/
src/domain/statistics/
```

Domain code may depend on other domain modules according to the architecture
checker, but must stay ROS-free.

### Application

Path:

```text
src/application/temporal_monitor/
```

Contains orchestration logic for the temporal monitor use case.

Application code coordinates domain analyzers and policies, but should not own
ROS message types or ROS node lifecycle.

### Presentation

Path:

```text
src/presentation/
```

Contains output formatting:

```text
src/presentation/render/
src/presentation/report/
```

Presentation code formats diagnostics, decisions, reports, summaries, and JSON.

### Platform

Path:

```text
src/platform/
```

Contains low-level utilities such as atomic file writing.

Platform code should remain small and dependency-light.

### ROS2 adapters

Path:

```text
src/adapters/ros2/
```

Contains ROS2-specific conversions and lookup adapters.

ROS2 adapters are allowed to depend on ROS message types and TF APIs.

### ROS2 apps

Path:

```text
src/apps/ros2/
```

Contains executable ROS2 nodes and node wiring.

Nodes should subscribe, convert, call application/domain code, publish results,
and avoid owning core policy logic.

### Tools

Path:

```text
tools/ros2/
```

Contains developer/demo tools such as fake publishers, fault injectors, and probes.

Tools are not production domain code.

### ROS1 adapter

Path:

```text
ros1/causal_slam_ros1/
```

Contains the ROS1 Noetic compatibility adapter. It is intentionally outside the
shared ROS-free core.

## Architecture checks

The project uses explicit checks:

```bash
./scripts/check_arch_deps.py
./scripts/check_config_params.py
```

`check_arch_deps.py` verifies:

- internal include dependency direction;
- ROS-facing includes are only used in ROS-facing modules;
- domain/application boundaries do not accidentally depend on ROS.

`check_config_params.py` verifies that ROS2 parameter declarations stay in sync
with `config/temporal_gate.yaml`.

## Quality gates

Before considering architecture changes valid, run:

```bash
./scripts/check_arch_deps.py
./scripts/check_config_params.py
git diff --check
make build
make test
make smoke
```

## Known compromises

Current known compromises:

- test directories still use older module names and may be migrated later;
- C++ namespaces have not been renamed to match the physical folder layout;
- the architecture checker still uses some historical logical module names;
- diagnostics and policy boundaries may be refined further so diagnostics owns
  evidence/health while policy owns decisions.