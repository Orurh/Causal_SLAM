#!/usr/bin/env python3

from pathlib import Path
import re
import sys
from collections import defaultdict

SOURCE_DIRS = [Path("src")]
EXTS = {".h", ".hpp", ".cc", ".cpp", ".cxx"}

# Supports both:
#   #include "foo/bar.h"
#   #include <foo/bar.hpp>
INCLUDE_RE = re.compile(r'^\s*#include\s+["<]([^">]+)[">]')


def module_of(path: str) -> str:
    parts = Path(path).parts
    if len(parts) >= 2 and parts[0] == "src":
        return parts[1]
    return parts[0] if parts else "unknown"


def normalize_internal_include(include: str) -> str | None:
    p = Path("src") / include
    if p.exists():
        return str(p)
    return None


allowed_edges = {
    ("coverage", "core"),
    ("lidar", "core"),
    ("ros_adapters", "core"),
    ("pipeline", "coverage"),
    ("pipeline", "diagnostics"),
    ("pipeline", "lidar"),
    ("pipeline", "model"),
    ("pipeline", "pointcloud"),
    ("pipeline", "statistics"),
    ("pipeline", "telemetry"),
    ("pointcloud", "core"),
    ("ros_adapters", "pointcloud"),

    ("model", "coverage"),
    ("model", "lidar"),
    ("model", "telemetry"),

    ("policy", "telemetry"),

    ("diagnostics", "model"),
    ("diagnostics", "telemetry"),
    ("diagnostics", "policy"),

    ("statistics", "model"),
    ("statistics", "telemetry"),

    ("render", "diagnostics"),
    ("render", "policy"),
    ("render", "statistics"),

    ("nodes", "coverage"),
    ("nodes", "diagnostics"),
    ("nodes", "lidar"),
    ("nodes", "model"),
    ("nodes", "policy"),
    ("nodes", "pipeline"),
    ("nodes", "render"),
    ("nodes", "ros_adapters"),
    ("nodes", "statistics"),
    ("nodes", "telemetry"),
}

forbidden_edges = {
    ("telemetry", "coverage"),
    ("telemetry", "lidar"),
    ("telemetry", "diagnostics"),
    ("telemetry", "statistics"),
    ("telemetry", "render"),
    ("telemetry", "nodes"),

    ("model", "diagnostics"),
    ("model", "statistics"),
    ("model", "render"),
    ("model", "nodes"),

    ("policy", "nodes"),
    ("policy", "render"),
    ("policy", "statistics"),
    ("policy", "diagnostics"),
    ("diagnostics", "render"),
    ("diagnostics", "nodes"),

    ("statistics", "diagnostics"),
    ("statistics", "render"),
    ("statistics", "nodes"),

    ("render", "nodes"),
}

ros_external_prefixes = (
    "rclcpp/",
    "rclcpp_components/",
    "rclcpp_action/",
    "sensor_msgs/",
    "std_msgs/",
    "builtin_interfaces/",
    "geometry_msgs/",
    "nav_msgs/",
    "tf2/",
    "tf2_ros/",
    "tf2_msgs/",
    "message_filters/",
)

ros_allowed_modules = {
    "nodes",
    "ros_adapters",
}


def is_ros_external_include(include: str) -> bool:
    return include.startswith(ros_external_prefixes)


module_edges: dict[str, set[str]] = defaultdict(set)
violations: list[str] = []

for base in SOURCE_DIRS:
    for path in base.rglob("*"):
        if path.suffix not in EXTS or not path.is_file():
            continue

        src_file = str(path)
        src_module = module_of(src_file)

        for line_number, line in enumerate(path.read_text(errors="replace").splitlines(), start=1):
            match = INCLUDE_RE.match(line)
            if not match:
                continue

            include = match.group(1)

            if is_ros_external_include(include) and src_module not in ros_allowed_modules:
                violations.append(
                    f'{src_file}:{line_number}: #include <{include}> is ROS-facing and is only allowed in '
                    f'{sorted(ros_allowed_modules)}; current module is "{src_module}"'
                )

            dst_file = normalize_internal_include(include)
            if dst_file is None:
                continue

            dst_module = module_of(dst_file)
            if src_module == dst_module:
                continue

            module_edges[src_module].add(dst_module)

            edge = (src_module, dst_module)
            if edge in forbidden_edges:
                violations.append(
                    f'{src_file}:{line_number}: #include "{include}" creates forbidden dependency '
                    f'{src_module} -> {dst_module}'
                )

            if edge not in allowed_edges:
                violations.append(
                    f'{src_file}:{line_number}: #include "{include}" creates unapproved dependency '
                    f'{src_module} -> {dst_module}'
                )

print("Module dependencies:")
for src in sorted(module_edges):
    for dst in sorted(module_edges[src]):
        print(f"  {src} -> {dst}")

if violations:
    print("\nArchitecture dependency violations:", file=sys.stderr)
    for violation in violations:
        print(f"  {violation}", file=sys.stderr)
    sys.exit(1)

print("\nArchitecture dependency check: OK")
