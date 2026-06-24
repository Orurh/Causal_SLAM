#!/usr/bin/env python3

from __future__ import annotations

import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PARAMETER_LOADER = REPO_ROOT / "src/apps/ros2/temporal_monitor_node_parameters.cc"
TEMPORAL_GATE_YAML = REPO_ROOT / "config/temporal_gate.yaml"

DECLARE_PARAMETER_RE = re.compile(
    r'declare_parameter\s*<[^>]+>\s*\(\s*"([^"]+)"',
    re.MULTILINE,
)


def extract_declared_parameters(source: str) -> set[str]:
    return set(DECLARE_PARAMETER_RE.findall(source))


def extract_temporal_monitor_yaml_parameters(yaml_text: str) -> set[str]:
    parameters: set[str] = set()

    in_temporal_monitor = False
    in_ros_parameters = False

    for raw_line in yaml_text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        stripped = line.strip()

        if not stripped:
            continue

        if re.match(r"^[A-Za-z0-9_]+:\s*$", line):
            in_temporal_monitor = stripped == "temporal_monitor:"
            in_ros_parameters = False
            continue

        if in_temporal_monitor and re.match(r"^\s{2}ros__parameters:\s*$", line):
            in_ros_parameters = True
            continue

        if in_temporal_monitor and in_ros_parameters:
            match = re.match(r"^\s{4}([A-Za-z0-9_]+):", line)
            if match:
                parameters.add(match.group(1))
                continue

            if re.match(r"^\s{0,2}[A-Za-z0-9_]+:\s*$", line):
                break

    return parameters


def main() -> int:
    declared = extract_declared_parameters(PARAMETER_LOADER.read_text(encoding="utf-8"))
    configured = extract_temporal_monitor_yaml_parameters(
        TEMPORAL_GATE_YAML.read_text(encoding="utf-8")
    )

    missing_in_yaml = sorted(declared - configured)

    if missing_in_yaml:
        print("Config parameter drift detected.")
        print()
        print(f"Parameter loader: {PARAMETER_LOADER.relative_to(REPO_ROOT)}")
        print(f"YAML config:       {TEMPORAL_GATE_YAML.relative_to(REPO_ROOT)}")
        print()
        print("Declared in C++ but missing in temporal_monitor.ros__parameters:")
        for name in missing_in_yaml:
            print(f"  - {name}")
        return 1

    print("Config parameter check: OK")
    print(f"Declared parameters: {len(declared)}")
    print(f"YAML parameters:     {len(configured)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())