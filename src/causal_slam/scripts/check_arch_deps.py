#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]

CODE_SUFFIXES = {".h", ".hpp", ".cc", ".cpp"}

INCLUDE_RE = re.compile(r'^\s*#include\s+[<"]([^>"]+)[>"]')


# Physical paths are transitional. Logical modules are the stable architecture
# units checked by this script.
#
# Rule order matters: more specific prefixes must go before generic ones.
PATH_MODULE_RULES: tuple[tuple[tuple[str, ...], str], ...] = (
    (("src", "adapters", "ros2", "support"), "ros_support"),
    (("src", "adapters", "ros2"), "ros_adapters"),
    (("src", "apps", "ros2"), "nodes"),

    (("src", "application", "temporal_monitor", "temporal_monitor_runtime_defaults.cc"), "config"),
    (("src", "application", "temporal_monitor", "temporal_monitor_runtime_defaults.h"), "config"),
    (("src", "application", "temporal_monitor"), "pipeline"),

    (("src", "presentation", "report"), "report"),
    (("src", "presentation", "render"), "render"),

    (("src", "domain", "time"), "core"),

    (("ros1",), "ros1_adapter"),
)

INCLUDE_MODULE_RULES: tuple[tuple[str, str], ...] = (
    ("adapters/ros2/support/", "ros_support"),
    ("adapters/ros2/", "ros_adapters"),
    ("apps/ros2/", "nodes"),

    ("application/temporal_monitor/temporal_monitor_runtime_defaults.", "config"),
    ("application/temporal_monitor/", "pipeline"),

    ("presentation/report/", "report"),
    ("presentation/render/", "render"),

    ("domain/time/", "core"),
)

INTERNAL_MODULES = {
    "benchmarks",
    "config",
    "core",
    "coverage",
    "diagnostics",
    "lidar",
    "model",
    "nodes",
    "pipeline",
    "platform",
    "pointcloud",
    "policy",
    "render",
    "report",
    "ros1_adapter",
    "ros_adapters",
    "ros_support",
    "statistics",
    "telemetry",
    "transform",
}

ALLOWED_DEPS = {
    # Foundation.
    ("coverage", "core"),
    ("lidar", "core"),
    ("pointcloud", "core"),
    ("transform", "telemetry"),

    # Domain/model.
    ("diagnostics", "model"),
    ("diagnostics", "policy"),
    ("diagnostics", "telemetry"),

    ("model", "coverage"),
    ("model", "lidar"),
    ("model", "telemetry"),
    ("model", "transform"),

    ("policy", "telemetry"),

    ("statistics", "model"),
    ("statistics", "telemetry"),

    # Application/config.
    ("config", "pipeline"),

    ("pipeline", "coverage"),
    ("pipeline", "diagnostics"),
    ("pipeline", "lidar"),
    ("pipeline", "model"),
    ("pipeline", "pointcloud"),
    ("pipeline", "statistics"),
    ("pipeline", "telemetry"),
    ("pipeline", "transform"),

    # Presentation.
    ("report", "coverage"),
    ("report", "diagnostics"),
    ("report", "lidar"),
    ("report", "policy"),
    ("report", "statistics"),
    ("report", "telemetry"),
    ("report", "transform"),

    ("render", "diagnostics"),
    ("render", "policy"),
    ("render", "report"),
    ("render", "statistics"),

    # ROS2 app.
    ("nodes", "config"),
    ("nodes", "coverage"),
    ("nodes", "diagnostics"),
    ("nodes", "lidar"),
    ("nodes", "model"),
    ("nodes", "pipeline"),
    ("nodes", "platform"),
    ("nodes", "policy"),
    ("nodes", "render"),
    ("nodes", "ros_adapters"),
    ("nodes", "ros_support"),
    ("nodes", "statistics"),
    ("nodes", "telemetry"),

    # ROS2 adapters.
    ("ros_adapters", "pointcloud"),
    ("ros_adapters", "transform"),

    # Benchmarks/tools.
    ("benchmarks", "pointcloud"),

    # ROS1 Noetic adapter boundary.
    # It is intentionally outside the shared ROS-free core.
    ("ros1_adapter", "coverage"),
    ("ros1_adapter", "diagnostics"),
    ("ros1_adapter", "lidar"),
    ("ros1_adapter", "pipeline"),
    ("ros1_adapter", "pointcloud"),
    ("ros1_adapter", "policy"),
    ("ros1_adapter", "render"),
    ("ros1_adapter", "telemetry"),
    ("ros1_adapter", "transform"),
}

ROS_FACING_INCLUDE_PREFIXES = (
    "rclcpp/",
    "builtin_interfaces/msg/",
    "sensor_msgs/msg/",
    "std_msgs/msg/",
    "geometry_msgs/msg/",
    "nav_msgs/msg/",
    "tf2/",
    "tf2_ros/",

    # ROS1 includes.
    "ros/",
    "sensor_msgs/",
    "std_msgs/",
    "geometry_msgs/",
    "nav_msgs/",
    "tf/",
)

ROS_FACING_ALLOWED_MODULES = {
    "nodes",
    "ros_adapters",
    "ros_support",
    "ros1_adapter",
}


def startswith_parts(parts: tuple[str, ...], prefix: tuple[str, ...]) -> bool:
    return len(parts) >= len(prefix) and parts[: len(prefix)] == prefix


def iter_code_files():
    for root in (ROOT / "src", ROOT / "tests", ROOT / "ros1"):
        if not root.exists():
            continue

        for path in root.rglob("*"):
            if path.suffix in CODE_SUFFIXES:
                yield path


def module_for_path(path: Path) -> str | None:
    rel = path.relative_to(ROOT)
    parts = rel.parts

    for prefix, module in PATH_MODULE_RULES:
        if startswith_parts(parts, prefix):
            return module

    if parts and parts[0] == "src" and len(parts) >= 2:
        return parts[1]

    if parts and parts[0] == "tests" and len(parts) >= 2:
        return parts[1]

    return None


def internal_target_module(include: str) -> str | None:
    for prefix, module in INCLUDE_MODULE_RULES:
        if include.startswith(prefix):
            return module

    first = include.split("/", 1)[0]

    if first in INTERNAL_MODULES:
        return first

    return None


def is_ros_facing_include(include: str) -> bool:
    return include.startswith(ROS_FACING_INCLUDE_PREFIXES)


def includes_of(path: Path):
    text = path.read_text(errors="replace")

    for line_no, line in enumerate(text.splitlines(), start=1):
        match = INCLUDE_RE.match(line)
        if match:
            yield line_no, match.group(1)


def is_allowed_dependency(source_module: str, target_module: str) -> bool:
    if source_module == target_module:
        return True

    # core is the innermost shared layer. Outer modules may depend on it,
    # but core must not depend outward.
    if target_module == "core":
        return True

    return (source_module, target_module) in ALLOWED_DEPS


def main() -> int:
    deps: set[tuple[str, str]] = set()
    violations: list[str] = []

    for path in iter_code_files():
        source_module = module_for_path(path)

        if source_module is None:
            continue

        rel = path.relative_to(ROOT)

        for line_no, include in includes_of(path):
            if is_ros_facing_include(include) and source_module not in ROS_FACING_ALLOWED_MODULES:
                violations.append(
                    f'{rel}:{line_no}: #include <{include}> is ROS-facing '
                    f'and is only allowed in {sorted(ROS_FACING_ALLOWED_MODULES)}; '
                    f'current module is "{source_module}"'
                )

            target_module = internal_target_module(include)
            if target_module is None:
                continue

            if source_module != target_module:
                deps.add((source_module, target_module))

            if not is_allowed_dependency(source_module, target_module):
                violations.append(
                    f'{rel}:{line_no}: #include "{include}" creates unapproved '
                    f"dependency {source_module} -> {target_module}"
                )

    print("Module dependencies:")
    for source, target in sorted(deps):
        print(f"  {source} -> {target}")

    if violations:
        print()
        print("Architecture dependency violations:")
        for violation in violations:
            print(f"  {violation}")
        return 1

    print()
    print("Architecture dependency check: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())