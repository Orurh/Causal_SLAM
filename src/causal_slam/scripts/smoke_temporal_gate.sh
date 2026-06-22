#!/usr/bin/env bash
set -Ee -o pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"

set -u

RUN_ID="run_$$_$(date +%s%N)"
PIDS=()
LOG_FILES=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      # Processes are started with setsid, so kill the whole process group.
      kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
    fi
  done

  sleep 0.5

  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -KILL -- "-${pid}" >/dev/null 2>&1 || true
    fi
  done

  wait >/dev/null 2>&1 || true
}

trap cleanup EXIT

start_process() {
  local name="$1"
  shift

  local log_file="/tmp/causal_slam_smoke_${RUN_ID}_${name}.log"
  LOG_FILES+=("${log_file}")

  echo "starting ${name}, log=${log_file}"

  setsid "$@" >"${log_file}" 2>&1 &
  PIDS+=("$!")
}

read_topic_data_once() {
  local topic="$1"

  timeout 5s ros2 topic echo "${topic}" --once 2>/dev/null \
    | awk -F': ' '/data:/ {print $2; exit}'
}

wait_for_topic_value() {
  local topic="$1"
  local expected="$2"
  local deadline_seconds="$3"

  local start
  start="$(date +%s)"

  while true; do
    local value
    value="$(read_topic_data_once "${topic}" || true)"

    if [[ "${value}" == "${expected}" ]]; then
      echo "${value}"
      return 0
    fi

    local now
    now="$(date +%s)"

    if (( now - start >= deadline_seconds )); then
      echo "FAIL: ${topic}: expected '${expected}', last value '${value}'" >&2
      echo "Smoke logs:" >&2
      printf '  %s\n' "${LOG_FILES[@]}" >&2
      return 1
    fi

    sleep 0.5
  done
}

expect_topic_eq() {
  local name="$1"
  local topic="$2"
  local expected="$3"

  local actual
  actual="$(wait_for_topic_value "${topic}" "${expected}" 30)"

  echo "OK: ${name}=${actual}"
}

expect_topic_contains() {
  local name="$1"
  local topic="$2"
  local expected_part="$3"
  local deadline_seconds="30"

  local start
  start="$(date +%s)"

  while true; do
    local value
    value="$(read_topic_data_once "${topic}" || true)"

    if [[ "${value}" == *"${expected_part}"* ]]; then
      echo "OK: ${name} contains ${expected_part} | value=${value}"
      return 0
    fi

    local now
    now="$(date +%s)"

    if (( now - start >= deadline_seconds )); then
      echo "FAIL: ${topic}: expected value containing '${expected_part}', last value '${value}'" >&2
      echo "Smoke logs:" >&2
      printf '  %s\n' "${LOG_FILES[@]}" >&2
      return 1
    fi

    sleep 0.5
  done
}

run_case() {
  local case_name="$1"
  local imu_period_ms="$2"
  local expected_allowed="$3"
  local expected_health="$4"
  local expected_reason="$5"
  local expected_fault_reasons="$6"

  local case_id="${RUN_ID}/${case_name}"
  local input_ns="/causal_slam_smoke/${case_id}/input"
  local output_ns="/causal_slam_smoke/${case_id}/output"

  local imu_topic="${input_ns}/imu"
  local lidar_topic="${input_ns}/points"
  local allowed_topic="${output_ns}/map_update_allowed"
  local health_topic="${output_ns}/temporal_health"
  local reason_topic="${output_ns}/map_update_reason"
  local fault_reasons_topic="${output_ns}/fault_reasons"

  echo
  echo "== ${case_name} =="
  echo "run_id=${RUN_ID}"
  echo "imu_period_ms=${imu_period_ms}"
  echo "imu_topic=${imu_topic}"
  echo "lidar_topic=${lidar_topic}"
  echo "allowed_topic=${allowed_topic}"

  PIDS=()
  LOG_FILES=()

  start_process "${case_name}_monitor" \
    ros2 run causal_slam temporal_monitor_node --ros-args \
      -p summary_period_ms:=500.0 \
      -p lidar_topic:="${lidar_topic}" \
      -p imu_topic:="${imu_topic}" \
      -p expected_imu_period_ms:=20.0 \
      -p imu_gap_threshold_ms:=500.0 \
      -p lidar_gap_threshold_ms:=1000.0 \
      -p map_update_allowed_topic:="${allowed_topic}" \
      -p temporal_health_topic:="${health_topic}" \
      -p map_update_reason_topic:="${reason_topic}" \
      -p fault_reasons_topic:="${fault_reasons_topic}"

  sleep 0.5

  start_process "${case_name}_imu" \
    ros2 run causal_slam fake_imu_publisher_node --ros-args \
      -p imu_topic:="${imu_topic}" \
      -p period_ms:="${imu_period_ms}"

  if [[ "${case_name}" == "healthy_pipeline" ]]; then
    sleep 4.0
  else
    sleep 2.0
  fi

  start_process "${case_name}_lidar" \
    ros2 run causal_slam fake_lidar_publisher_node --ros-args \
      -p lidar_topic:="${lidar_topic}" \
      -p period_ms:=100.0 \
      -p point_count:=5 \
      -p include_xyz_fields:=true \
      -p time_field_mode:=offset_time_uint32 \
      -p scan_duration_ms:=100.0

  expect_topic_eq "${case_name}.map_update_allowed" \
    "${allowed_topic}" "${expected_allowed}"

  expect_topic_eq "${case_name}.temporal_health" \
    "${health_topic}" "${expected_health}"

  expect_topic_eq "${case_name}.map_update_reason" \
    "${reason_topic}" "${expected_reason}"

  if [[ "${expected_fault_reasons}" == "none" ]]; then
    expect_topic_eq "${case_name}.fault_reasons" \
      "${fault_reasons_topic}" "none"
  else
    expect_topic_contains "${case_name}.fault_reasons" \
      "${fault_reasons_topic}" "${expected_fault_reasons}"
  fi

  cleanup
  PIDS=()
  LOG_FILES=()
}

run_case "healthy_pipeline" "20.0" "true" "OK" "temporal_health_ok" "none"
run_case "degraded_imu_pipeline" "200.0" "false" "DEGRADED" "temporal_health_degraded" "imu_window_incomplete"

echo
echo "Causal-SLAM temporal gate smoke test: OK"
