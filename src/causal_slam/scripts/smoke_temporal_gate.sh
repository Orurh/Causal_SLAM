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

SMOKE_LOG_DIR="${CAUSAL_SLAM_SMOKE_LOG_DIR:-${WS_DIR}/log/smoke}"
SMOKE_HTML_DIR="${CAUSAL_SLAM_SMOKE_HTML_DIR:-${WS_DIR}/log/smoke}"

mkdir -p "${SMOKE_LOG_DIR}" "${SMOKE_HTML_DIR}"

SMOKE_SUMMARY_PERIOD_MS="500.0"
SMOKE_EXPECTED_IMU_PERIOD_MS="20.0"
SMOKE_IMU_GAP_THRESHOLD_MS="500.0"
SMOKE_LIDAR_GAP_THRESHOLD_MS="1000.0"

SMOKE_HEALTHY_IMU_PERIOD_MS="20.0"
SMOKE_DEGRADED_IMU_PERIOD_MS="200.0"
SMOKE_LAGGED_IMU_TIMESTAMP_SHIFT_MS="-150.0"
SMOKE_DROPPED_IMU_DROP_EVERY_N="2"
SMOKE_DROPPED_IMU_GAP_THRESHOLD_MS="30.0"

SMOKE_LIDAR_PERIOD_MS="100.0"
SMOKE_LIDAR_POINT_COUNT="5"
SMOKE_LIDAR_SCAN_DURATION_MS="100.0"

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

  local log_file="${SMOKE_LOG_DIR}/causal_slam_smoke_${RUN_ID}_${name}.log"
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

expect_html_report() {
  local case_name="$1"
  local path="$2"
  local deadline_seconds="30"

  local start
  start="$(date +%s)"

  while true; do
    if [[ -s "${path}" ]] && grep -q "Causal-SLAM Temporal Report" "${path}"; then
      echo "OK: ${case_name}.html_report=${path}"
      return 0
    fi

    local now
    now="$(date +%s)"

    if (( now - start >= deadline_seconds )); then
      echo "FAIL: ${case_name}: expected HTML report at '${path}'" >&2
      echo "Smoke logs:" >&2
      printf '  %s\n' "${LOG_FILES[@]}" >&2
      return 1
    fi

    sleep 0.5
  done
}

expect_topic_has_message() {
  local name="$1"
  local topic="$2"

  if timeout 10s ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    echo "OK: ${name} received message"
    return 0
  fi

  echo "FAIL: ${topic}: expected at least one message" >&2
  echo "Smoke logs:" >&2
  printf '  %s\n' "${LOG_FILES[@]}" >&2
  return 1
}

expect_topic_has_no_message() {
  local name="$1"
  local topic="$2"

  if timeout 3s ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    echo "FAIL: ${topic}: expected no messages, but one was received" >&2
    echo "Smoke logs:" >&2
    printf '  %s\n' "${LOG_FILES[@]}" >&2
    return 1
  fi

  echo "OK: ${name} received no messages"
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
  local tf_monitoring_enabled="${7:-false}"
  local static_tf_enabled="${8:-false}"
  local html_report_enabled="${9:-false}"
  local imu_timestamp_shift_ms="${10:-0.0}"
  local imu_gap_threshold_ms="${11:-${SMOKE_IMU_GAP_THRESHOLD_MS}}"
  local imu_drop_every_n="${12:-0}"
  local imu_fault_injector_enabled="${13:-false}"
  local html_report_path="${SMOKE_HTML_DIR}/causal_slam_smoke_${RUN_ID}_${case_name}.html"

  local case_id="${RUN_ID}/${case_name}"
  local input_ns="/causal_slam_smoke/${case_id}/input"
  local output_ns="/causal_slam_smoke/${case_id}/output"

  local imu_topic="${input_ns}/imu"
  local lidar_topic="${input_ns}/points"
  local checked_lidar_topic="${output_ns}/points_checked"
  local allowed_topic="${output_ns}/map_update_allowed"
  local health_topic="${output_ns}/temporal_health"
  local reason_topic="${output_ns}/map_update_reason"
  local fault_reasons_topic="${output_ns}/fault_reasons"
  local decision_json_topic="${output_ns}/map_update_decision_json"

  echo
  echo "== ${case_name} =="
  echo "run_id=${RUN_ID}"
  echo "imu_period_ms=${imu_period_ms}"
  echo "imu_timestamp_shift_ms=${imu_timestamp_shift_ms}"
  echo "imu_gap_threshold_ms=${imu_gap_threshold_ms}"
  echo "imu_drop_every_n=${imu_drop_every_n}"
  echo "imu_fault_injector_enabled=${imu_fault_injector_enabled}"
  echo "imu_topic=${imu_topic}"
  echo "lidar_topic=${lidar_topic}"
  echo "allowed_topic=${allowed_topic}"
  echo "tf_monitoring_enabled=${tf_monitoring_enabled}"
  echo "static_tf_enabled=${static_tf_enabled}"
  echo "html_report_enabled=${html_report_enabled}"

  PIDS=()
  LOG_FILES=()

  start_process "${case_name}_monitor" \
    ros2 run causal_slam temporal_monitor_node --ros-args \
      -p summary_period_ms:="${SMOKE_SUMMARY_PERIOD_MS}" \
      -p lidar_topic:="${lidar_topic}" \
      -p checked_lidar_topic:="${checked_lidar_topic}" \
      -p lidar_gate_mode:=drop_degraded \
      -p imu_topic:="${imu_topic}" \
      -p expected_imu_period_ms:="${SMOKE_EXPECTED_IMU_PERIOD_MS}" \
      -p imu_gap_threshold_ms:="${imu_gap_threshold_ms}" \
      -p lidar_gap_threshold_ms:="${SMOKE_LIDAR_GAP_THRESHOLD_MS}" \
      -p map_update_allowed_topic:="${allowed_topic}" \
      -p temporal_health_topic:="${health_topic}" \
      -p map_update_reason_topic:="${reason_topic}" \
      -p fault_reasons_topic:="${fault_reasons_topic}" \
      -p map_update_decision_json_topic:="${decision_json_topic}" \
      -p tf_monitoring_enabled:="${tf_monitoring_enabled}" \
      -p html_report_path:="${html_report_path}"

  sleep 0.5

  if [[ "${static_tf_enabled}" == "true" ]]; then
    start_process "${case_name}_static_tf" \
      ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 \
        --roll 0 --pitch 0 --yaw 0 \
        --frame-id odom \
        --child-frame-id lidar

    sleep 0.5
  fi

  local fake_imu_topic="${imu_topic}"

  if [[ "${imu_fault_injector_enabled}" == "true" ]]; then
    local raw_imu_topic="${input_ns}/imu_raw"
    fake_imu_topic="${raw_imu_topic}"

    start_process "${case_name}_imu_fault_injector" \
      ros2 run causal_slam imu_fault_injection_node --ros-args \
        -p input_topic:="${raw_imu_topic}" \
        -p output_topic:="${imu_topic}" \
        -p timestamp_shift_ms:="${imu_timestamp_shift_ms}" \
        -p drop_every_n:="${imu_drop_every_n}"

    sleep 0.5
  fi

  start_process "${case_name}_imu" \
    ros2 run causal_slam fake_imu_publisher_node --ros-args \
      -p imu_topic:="${fake_imu_topic}" \
      -p period_ms:="${imu_period_ms}" \
      -p timestamp_shift_ms:=0.0 \
      -p drop_every_n:=0

  if [[ "${case_name}" == "healthy_pipeline" ]]; then
    sleep 4.0
  else
    sleep 2.0
  fi

  start_process "${case_name}_lidar" \
    ros2 run causal_slam fake_lidar_publisher_node --ros-args \
      -p lidar_topic:="${lidar_topic}" \
      -p period_ms:="${SMOKE_LIDAR_PERIOD_MS}" \
      -p point_count:="${SMOKE_LIDAR_POINT_COUNT}" \
      -p include_xyz_fields:=true \
      -p time_field_mode:=offset_time_uint32 \
      -p scan_duration_ms:="${SMOKE_LIDAR_SCAN_DURATION_MS}"

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

  expect_topic_contains "${case_name}.decision_json.allowed" \
    "${decision_json_topic}" "\"allowed\":${expected_allowed}"

  expect_topic_contains "${case_name}.decision_json.health" \
    "${decision_json_topic}" "\"health\":\"${expected_health}\""

  expect_topic_contains "${case_name}.decision_json.scan_stamp" \
    "${decision_json_topic}" "\"scan_stamp_ns\":"

  expect_topic_contains "${case_name}.decision_json.frame_id" \
    "${decision_json_topic}" "\"frame_id\":\"lidar\""

  if [[ "${expected_allowed}" == "true" ]]; then
    expect_topic_has_message "${case_name}.checked_lidar.forward" \
      "${checked_lidar_topic}"
  else
    expect_topic_has_no_message "${case_name}.checked_lidar.block" \
      "${checked_lidar_topic}"
  fi

  if [[ "${html_report_enabled}" == "true" ]]; then
    expect_html_report "${case_name}" "${html_report_path}"
  fi

  cleanup
  PIDS=()
  LOG_FILES=()
}

run_case "healthy_pipeline" "${SMOKE_HEALTHY_IMU_PERIOD_MS}" "true" "OK" "temporal_health_ok" "none"
run_case "degraded_imu_pipeline" "${SMOKE_DEGRADED_IMU_PERIOD_MS}" "false" "DEGRADED" "temporal_health_degraded" "imu_window_incomplete"
run_case "lagged_imu_timestamp_pipeline" "${SMOKE_HEALTHY_IMU_PERIOD_MS}" "false" "DEGRADED" "temporal_health_degraded" "imu_window_incomplete" "false" "false" "false" "${SMOKE_LAGGED_IMU_TIMESTAMP_SHIFT_MS}" "${SMOKE_IMU_GAP_THRESHOLD_MS}" "0" "true"
run_case "dropped_imu_samples_pipeline" "${SMOKE_HEALTHY_IMU_PERIOD_MS}" "false" "DEGRADED" "temporal_health_degraded" "stream_timing_unstable" "false" "false" "false" "0.0" "${SMOKE_DROPPED_IMU_GAP_THRESHOLD_MS}" "${SMOKE_DROPPED_IMU_DROP_EVERY_N}" "true"
run_case "missing_tf_pipeline" "${SMOKE_HEALTHY_IMU_PERIOD_MS}" "false" "INVALID" "temporal_health_invalid" "tf_lookup_failed" "true"
run_case "healthy_tf_pipeline" "${SMOKE_HEALTHY_IMU_PERIOD_MS}" "true" "OK" "temporal_health_ok" "none" "true" "true" "true"

echo
echo "Causal-SLAM temporal gate smoke test: OK"
