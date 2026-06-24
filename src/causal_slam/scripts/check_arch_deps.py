#!/usr/bin/env python3

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]

INTERNAL_MODULES = {
    "core",
    "coverage",
    "diagnostics",
    "lidar",
    "model",
    "nodes",
    "pipeline",
    "pointcloud",
    "policy",
    "render",
    "ros_adapters",
    "statistics",
    "telemetry",
    "transform",
}

ALLOWED_DEPS = {
    ("nodes", "platform"),
    ("report", "transform"),
    ("report", "telemetry"),
    ("report", "policy"),
    ("report", "lidar"),
    ("report", "coverage"),
    ("report", "statistics"),
    ("report", "diagnostics"),
    ("render", "report"),
    ("config", "pipeline"),
    ("nodes", "config"),
    ("coverage", "core"),

    ("diagnostics", "model"),
    ("diagnostics", "policy"),
    ("diagnostics", "telemetry"),

    ("lidar", "core"),

    ("model", "coverage"),
    ("model", "lidar"),
    ("model", "telemetry"),
    ("model", "transform"),


    ("nodes", "coverage"),
    ("nodes", "diagnostics"),
    ("nodes", "lidar"),
    ("nodes", "model"),
    ("nodes", "pipeline"),
    ("nodes", "policy"),
    ("nodes", "render"),
    ("nodes", "ros_adapters"),
    ("nodes", "statistics"),
    ("nodes", "telemetry"),

    ("pipeline", "coverage"),
    ("pipeline", "diagnostics"),
    ("pipeline", "lidar"),
    ("pipeline", "model"),
    ("pipeline", "pointcloud"),
    ("pipeline", "statistics"),
    ("pipeline", "telemetry"),
    ("pipeline", "transform"),


    ("pointcloud", "core"),

    ("policy", "telemetry"),

    ("render", "diagnostics"),
    ("render", "policy"),
    ("render", "statistics"),

    ("benchmarks", "pointcloud"),
    ("ros_adapters", "pointcloud"),
    ("ros_adapters", "transform"),

    ("statistics", "model"),
    ("statistics", "telemetry"),

    ("transform", "telemetry"),

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
    "tf2_ros/",

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
    "ros1_adapter",
}

INCLUDE_RE = re.compile(r'^\s*#include\s+[<"]([^>"]+)[>"]')


def iter_code_files():
    roots = [
        ROOT / "src",
        ROOT / "tests",
        ROOT / "ros1",
    ]

    for root in roots:
        if not root.exists():
            continue

        for path in root.rglob("*"):
            if path.suffix in {".h", ".hpp", ".cc", ".cpp"}:
                yield path


def module_for_path(path: Path) -> str | None:
    rel = path.relative_to(ROOT)
    parts = rel.parts

    if not parts:
        return None

    if parts[0] == "src" and len(parts) >= 2:
        return parts[1]

    if parts[0] == "tests" and len(parts) >= 2:
        return parts[1]

    if parts[0] == "ros1":
        return "ros1_adapter"

    return None


def is_ros_facing_include(include: str) -> bool:
    return include.startswith(ROS_FACING_INCLUDE_PREFIXES)


def internal_target_module(include: str) -> str | None:
    first = include.split("/", 1)[0]

    if first in INTERNAL_MODULES:
        return first

    return None


def includes_of(path: Path):
    text = path.read_text(errors="replace")

    for line_no, line in enumerate(text.splitlines(), start=1):
        match = INCLUDE_RE.match(line)
        if match:
            yield line_no, match.group(1)


def main() -> int:
    deps: set[tuple[str, str]] = set()
    violations: list[str] = []

    for path in iter_code_files():
        source_module = module_for_path(path)

        if source_module is None:
            continue

        rel = path.relative_to(ROOT)

        for line_no, include in includes_of(path):
            if is_ros_facing_include(include):
                if source_module not in ROS_FACING_ALLOWED_MODULES:
                    violations.append(
                        f'{rel}:{line_no}: #include <{include}> is ROS-facing '
                        f'and is only allowed in {sorted(ROS_FACING_ALLOWED_MODULES)}; '
                        f'current module is "{source_module}"'
                    )

            target_module = internal_target_module(include)
            if target_module is None:
                continue

            if target_module == source_module:
                continue

            deps.add((source_module, target_module))

            if (source_module, target_module) not in ALLOWED_DEPS:
                violations.append(
                    f'{rel}:{line_no}: #include "{include}" creates unapproved '
                    f'dependency {source_module} -> {target_module}'
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
