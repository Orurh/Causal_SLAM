#!/usr/bin/env bash
set -euo pipefail

SAMPLES="${1:-20}"
SLEEP_SEC="${2:-1}"

for i in $(seq 1 "$SAMPLES"); do
  echo "=== sample $i ==="
  json="$(ros2 topic echo --full-length /causal_slam/map_update_decision_json --once || true)"
  echo "$json"

  if echo "$json" | grep -Eq 'imu_window_empty|imu_window_incomplete|imu_window_missing_suffix'; then
    echo "ERROR: false IMU window fault detected"
    exit 1
  fi

  sleep "$SLEEP_SEC"
done

echo "OK: no false IMU window faults detected in $SAMPLES samples"
