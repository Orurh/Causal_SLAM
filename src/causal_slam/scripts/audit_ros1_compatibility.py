#!/usr/bin/env python3

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TESTS = ROOT / "tests"

ROS2_INCLUDE_PREFIXES = (
    "rclcpp/",
    "rclcpp_components/",
    "sensor_msgs/msg/",
    "std_msgs/msg/",
    "builtin_interfaces/msg/",
    "geometry_msgs/msg/",
    "nav_msgs/msg/",
    "tf2/",
    "tf2_ros/",
)

ROS1_INCLUDE_PREFIXES = (
    "ros/",
    "sensor_msgs/",
    "std_msgs/",
    "geometry_msgs/",
    "nav_msgs/",
    "tf/",
)

CXX_RISK_PATTERNS = {
    "concepts": re.compile(r"\bconcept\b|#include\s*<concepts>"),
    "std_expected": re.compile(r"std::expected|#include\s*<expected>"),
    "std_format": re.compile(r"std::format|#include\s*<format>"),
    "std_print": re.compile(r"std::print|#include\s*<print>"),
    "std_ranges": re.compile(r"std::ranges|#include\s*<ranges>"),
    "designated_initializers": re.compile(r"\.\w+\s*="),
}

ROS_FREE_MODULES = {
    "core",
    "coverage",
    "diagnostics",
    "lidar",
    "model",
    "policy",
    "pointcloud",
    "pipeline",
    "render",
    "statistics",
    "telemetry",
    "transform",
}

ROS_BOUNDARY_MODULES = {
    "nodes",
    "ros_adapters",
}

def iter_code_files():
    for base in (SRC, TESTS):
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if path.suffix in {".h", ".hpp", ".cc", ".cpp"}:
                yield path

def module_of(path: Path) -> str:
    rel = path.relative_to(ROOT)
    parts = rel.parts

    if parts[0] == "src" and len(parts) >= 2:
        return parts[1]

    if parts[0] == "tests" and len(parts) >= 2:
        return "tests/" + parts[1]

    return "unknown"

def includes_of(text: str):
    result = []
    for line_no, line in enumerate(text.splitlines(), start=1):
        match = re.match(r'\s*#include\s+[<"]([^>"]+)[>"]', line)
        if match:
            result.append((line_no, match.group(1)))
    return result

def is_ros_include(include: str) -> bool:
    return include.startswith(ROS2_INCLUDE_PREFIXES) or include.startswith(ROS1_INCLUDE_PREFIXES)

def main() -> int:
    violations = []
    ros2_files = []
    ros_free_files = []
    cxx_risks = []

    for path in iter_code_files():
        rel = path.relative_to(ROOT)
        module = module_of(path)
        text = path.read_text(errors="replace")

        has_ros_include = False
        has_ros2_include = False

        for line_no, include in includes_of(text):
            if is_ros_include(include):
                has_ros_include = True

            if include.startswith(ROS2_INCLUDE_PREFIXES):
                has_ros2_include = True

                if module in ROS_FREE_MODULES:
                    violations.append(
                        f"{rel}:{line_no}: ROS2 include in ROS-free module '{module}': {include}"
                    )

        if has_ros2_include:
            ros2_files.append(str(rel))
        elif module in ROS_FREE_MODULES:
            ros_free_files.append(str(rel))

        for name, pattern in CXX_RISK_PATTERNS.items():
            if pattern.search(text):
                cxx_risks.append(f"{rel}: {name}")

    print("ROS1/Noetic compatibility audit")
    print()

    print("ROS-free reusable module files:")
    for item in sorted(ros_free_files):
        print(f"  {item}")

    print()
    print("ROS2-boundary files:")
    for item in sorted(ros2_files):
        print(f"  {item}")

    print()
    print("C++ feature risks for Noetic/catkin builds:")
    if cxx_risks:
        for item in sorted(set(cxx_risks)):
            print(f"  {item}")
    else:
        print("  none")

    print()
    if violations:
        print("Violations:")
        for violation in violations:
            print(f"  {violation}")
        return 1

    print("ROS-free core boundary: OK")
    return 0

if __name__ == "__main__":
    sys.exit(main())
