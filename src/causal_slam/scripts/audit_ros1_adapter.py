#!/usr/bin/env python3

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
ROS1_ROOT = ROOT / "ros1" / "causal_slam_ros1"

ROS2_FORBIDDEN_INCLUDES = (
    "rclcpp/",
    "sensor_msgs/msg/",
    "std_msgs/msg/",
    "builtin_interfaces/msg/",
    "geometry_msgs/msg/",
    "nav_msgs/msg/",
    "tf2_ros/",
)

ROS1_EXPECTED_INCLUDES = (
    "ros/",
    "sensor_msgs/",
    "std_msgs/",
)

def iter_code_files():
    if not ROS1_ROOT.exists():
        return

    for path in ROS1_ROOT.rglob("*"):
        if path.suffix in {".h", ".hpp", ".cc", ".cpp"}:
            yield path

def includes_of(text: str):
    for line_no, line in enumerate(text.splitlines(), start=1):
        match = re.match(r'\s*#include\s+[<"]([^>"]+)[>"]', line)
        if match:
            yield line_no, match.group(1)

def main() -> int:
    violations = []
    ros1_include_count = 0

    if not ROS1_ROOT.exists():
        print(f"ROS1 adapter directory not found: {ROS1_ROOT}")
        return 1

    for path in iter_code_files():
        rel = path.relative_to(ROOT)
        text = path.read_text(errors="replace")

        for line_no, include in includes_of(text):
            if include.startswith(ROS2_FORBIDDEN_INCLUDES):
                violations.append(
                    f"{rel}:{line_no}: forbidden ROS2 include in ROS1 adapter: {include}"
                )

            if include.startswith(ROS1_EXPECTED_INCLUDES):
                ros1_include_count += 1

    print("ROS1 adapter audit")
    print(f"  root: {ROS1_ROOT.relative_to(ROOT)}")
    print(f"  ROS1 include count: {ros1_include_count}")

    if violations:
      print()
      print("Violations:")
      for violation in violations:
          print(f"  {violation}")
      return 1

    print("ROS1 adapter boundary: OK")
    return 0

if __name__ == "__main__":
    sys.exit(main())
