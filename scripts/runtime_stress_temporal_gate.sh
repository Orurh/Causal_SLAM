#!/usr/bin/env bash
set -Eeuo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${PACKAGE_DIR}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

RUN_ID="runtime_stress_$$_$(date +%s%N)"
LOG_DIR="${CAUSAL_SLAM_RUNTIME_STRESS_LOG_DIR:-${WS_DIR}/log/runtime_stress}"
mkdir -p "${LOG_DIR}"

DURATION_SEC="${CAUSAL_SLAM_RUNTIME_STRESS_DURATION_SEC:-20}"
POINT_COUNT="${CAUSAL_SLAM_RUNTIME_STRESS_POINT_COUNT:-1000000}"
LIDAR_PERIOD_MS="${CAUSAL_SLAM_RUNTIME_STRESS_LIDAR_PERIOD_MS:-100.0}"
IMU_PERIOD_MS="${CAUSAL_SLAM_RUNTIME_STRESS_IMU_PERIOD_MS:-20.0}"
GATE_MODE="${CAUSAL_SLAM_RUNTIME_STRESS_GATE_MODE:-drop_degraded}"
PUBLISHER_ONLY="${CAUSAL_SLAM_RUNTIME_STRESS_PUBLISHER_ONLY:-0}"
QOS_RELIABILITY="${CAUSAL_SLAM_RUNTIME_STRESS_QOS_RELIABILITY:-best_effort}"
QOS_DEPTH="${CAUSAL_SLAM_RUNTIME_STRESS_QOS_DEPTH:-5}"

MIN_INPUT_HZ="${CAUSAL_SLAM_RUNTIME_STRESS_MIN_INPUT_HZ:-9.0}"
MIN_CHECKED_HZ="${CAUSAL_SLAM_RUNTIME_STRESS_MIN_CHECKED_HZ:-9.0}"
FAIL_ON_MONITOR_WARNINGS="${CAUSAL_SLAM_RUNTIME_STRESS_FAIL_ON_MONITOR_WARNINGS:-1}"
MONITOR_BAD_RE="${CAUSAL_SLAM_RUNTIME_STRESS_MONITOR_BAD_RE:-WARN|ERROR|blocked|dropped}"

LIDAR_HOLDBACK_ENABLED="${CAUSAL_SLAM_RUNTIME_STRESS_LIDAR_HOLDBACK_ENABLED:-true}"
LIDAR_HOLDBACK_TOLERANCE_MS="${CAUSAL_SLAM_RUNTIME_STRESS_LIDAR_HOLDBACK_TOLERANCE_MS:-10.0}"
LIDAR_HOLDBACK_MAX_PENDING="${CAUSAL_SLAM_RUNTIME_STRESS_LIDAR_HOLDBACK_MAX_PENDING:-32}"
IMU_BUFFER_RETENTION_MS="${CAUSAL_SLAM_RUNTIME_STRESS_IMU_BUFFER_RETENTION_MS:-5000.0}"

INPUT_NS="/causal_slam_runtime_stress/${RUN_ID}/input"
OUTPUT_NS="/causal_slam_runtime_stress/${RUN_ID}/output"

IMU_TOPIC="${INPUT_NS}/imu"
LIDAR_TOPIC="${INPUT_NS}/points"
CHECKED_LIDAR_TOPIC="${OUTPUT_NS}/points_checked"
ALLOWED_TOPIC="${OUTPUT_NS}/map_update_allowed"
HEALTH_TOPIC="${OUTPUT_NS}/temporal_health"
REASON_TOPIC="${OUTPUT_NS}/map_update_reason"
FAULT_REASONS_TOPIC="${OUTPUT_NS}/fault_reasons"
DECISION_JSON_TOPIC="${OUTPUT_NS}/map_update_decision_json"

MONITOR_LOG="${LOG_DIR}/${RUN_ID}_monitor.log"
IMU_LOG="${LOG_DIR}/${RUN_ID}_imu.log"
LIDAR_LOG="${LOG_DIR}/${RUN_ID}_lidar.log"
INPUT_HZ_LOG="${LOG_DIR}/${RUN_ID}_input_hz.log"
OUTPUT_HZ_LOG="${LOG_DIR}/${RUN_ID}_output_hz.log"
DECISION_JSON_LOG="${LOG_DIR}/${RUN_ID}_decision_json.log"

PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
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

truthy() {
  case "$1" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

start_process() {
  local name="$1"
  shift

  local log_file="${LOG_DIR}/${RUN_ID}_${name}.log"
  echo "starting ${name}, log=${log_file}"

  setsid "$@" >"${log_file}" 2>&1 &
  PIDS+=("$!")
}

start_logged_timeout_process() {
  local name="$1"
  local log_file="$2"
  shift 2

  echo "starting ${name}, log=${log_file}"

  setsid timeout "${DURATION_SEC}s" "$@" >"${log_file}" 2>&1 &
  local pid="$!"
  PIDS+=("${pid}")

  echo "${pid}"
}

extract_last_total_hz() {
  local log_file="$1"
  local label="$2"

  grep "PointCloudRateProbe | label=${label}" "${log_file}" \
    | tail -1 \
    | sed -n 's/.*total_hz=\([0-9.]*\).*/\1/p'
}

check_float_gte() {
  local actual="$1"
  local minimum="$2"
  local label="$3"

  if [[ -z "${actual}" ]]; then
    echo "ERROR: ${label}: missing numeric value"
    return 1
  fi

  if ! awk -v actual="${actual}" -v minimum="${minimum}" \
      'BEGIN { exit !((actual + 0.0) >= (minimum + 0.0)) }'; then
    echo "ERROR: ${label}: actual=${actual}, minimum=${minimum}"
    return 1
  fi

  echo "OK: ${label}: actual=${actual}, minimum=${minimum}"
}

echo "== Causal-SLAM runtime stress =="
echo "run_id=${RUN_ID}"
echo "duration_sec=${DURATION_SEC}"
echo "point_count=${POINT_COUNT}"
echo "lidar_period_ms=${LIDAR_PERIOD_MS}"
echo "imu_period_ms=${IMU_PERIOD_MS}"
echo "gate_mode=${GATE_MODE}"
echo "publisher_only=${PUBLISHER_ONLY}"
echo "qos_reliability=${QOS_RELIABILITY}"
echo "qos_depth=${QOS_DEPTH}"
echo "min_input_hz=${MIN_INPUT_HZ}"
echo "min_checked_hz=${MIN_CHECKED_HZ}"
echo "fail_on_monitor_warnings=${FAIL_ON_MONITOR_WARNINGS}"
echo "lidar_holdback_enabled=${LIDAR_HOLDBACK_ENABLED}"
echo "lidar_holdback_tolerance_ms=${LIDAR_HOLDBACK_TOLERANCE_MS}"
echo "lidar_holdback_max_pending=${LIDAR_HOLDBACK_MAX_PENDING}"
echo "imu_buffer_retention_ms=${IMU_BUFFER_RETENTION_MS}"
echo "lidar_topic=${LIDAR_TOPIC}"
echo "checked_lidar_topic=${CHECKED_LIDAR_TOPIC}"
echo "decision_json_topic=${DECISION_JSON_TOPIC}"
echo "log_dir=${LOG_DIR}"

if ! truthy "${PUBLISHER_ONLY}"; then
  start_process monitor \
    ros2 run causal_slam temporal_monitor_node --ros-args \
      -p runtime_profile:=minimal \
      -p imu_topic:="${IMU_TOPIC}" \
      -p lidar_topic:="${LIDAR_TOPIC}" \
      -p checked_lidar_topic:="${CHECKED_LIDAR_TOPIC}" \
      -p map_update_allowed_topic:="${ALLOWED_TOPIC}" \
      -p temporal_health_topic:="${HEALTH_TOPIC}" \
      -p map_update_reason_topic:="${REASON_TOPIC}" \
      -p fault_reasons_topic:="${FAULT_REASONS_TOPIC}" \
      -p map_update_decision_json_topic:="${DECISION_JSON_TOPIC}" \
      -p lidar_gate_mode:="${GATE_MODE}" \
      -p lidar_qos_reliability:="${QOS_RELIABILITY}" \
      -p lidar_qos_depth:="${QOS_DEPTH}" \
      -p checked_lidar_qos_reliability:="${QOS_RELIABILITY}" \
      -p checked_lidar_qos_depth:="${QOS_DEPTH}" \
      -p lidar_holdback_enabled:="${LIDAR_HOLDBACK_ENABLED}" \
      -p lidar_holdback_tolerance_ms:="${LIDAR_HOLDBACK_TOLERANCE_MS}" \
      -p lidar_holdback_max_pending:="${LIDAR_HOLDBACK_MAX_PENDING}" \
      -p imu_buffer_retention_ms:="${IMU_BUFFER_RETENTION_MS}" \
      -p lidar_point_time_mode:=explicit \
      -p lidar_point_time_field:=offset_time \
      -p lidar_point_time_interpretation:=relative \
      -p lidar_point_time_unit:=nanoseconds \
      -p tf_monitoring_enabled:=false \
      -p expected_imu_period_ms:="${IMU_PERIOD_MS}" \
      -p imu_gap_threshold_ms:=500.0 \
      -p lidar_gap_threshold_ms:=1000.0 \
      -p max_missing_prefix_ms:=40.0 \
      -p max_missing_suffix_ms:=40.0 \
      -p max_internal_gap_ms:=100.0

  start_process imu \
    ros2 run causal_slam fake_imu_publisher_node --ros-args \
      -p imu_topic:="${IMU_TOPIC}" \
      -p period_ms:="${IMU_PERIOD_MS}"
fi

start_process lidar \
  ros2 run causal_slam fake_lidar_publisher_node --ros-args \
    -p lidar_topic:="${LIDAR_TOPIC}" \
    -p frame_id:=lidar \
    -p period_ms:="${LIDAR_PERIOD_MS}" \
    -p point_count:="${POINT_COUNT}" \
    -p include_xyz_fields:=true \
    -p time_field_mode:=offset_time_uint32 \
    -p scan_duration_ms:="${LIDAR_PERIOD_MS}" \
    -p perf_metrics_enabled:=true \
    -p perf_metrics_period_ms:=2000.0 \
    -p qos_reliability:="${QOS_RELIABILITY}" \
    -p qos_depth:="${QOS_DEPTH}"

sleep 3

if ! truthy "${PUBLISHER_ONLY}"; then
  echo
  echo "reading current temporal gate status..."
  timeout 10s ros2 topic echo --once "${ALLOWED_TOPIC}" || true
  timeout 10s ros2 topic echo --once "${HEALTH_TOPIC}" || true
fi

echo
echo "measuring topic rates for ${DURATION_SEC}s..."

setsid timeout "${DURATION_SEC}s" ros2 run causal_slam point_cloud_rate_probe_node --ros-args \
  -r __node:="input_rate_probe_${RUN_ID}" \
  -p topic:="${LIDAR_TOPIC}" \
  -p label:=input_lidar \
  -p summary_period_ms:=2000.0 \
  -p qos_reliability:="${QOS_RELIABILITY}" \
  -p qos_depth:="${QOS_DEPTH}" >"${INPUT_HZ_LOG}" 2>&1 &
HZ_INPUT_PID="$!"
PIDS+=("${HZ_INPUT_PID}")

if ! truthy "${PUBLISHER_ONLY}"; then
  setsid timeout "${DURATION_SEC}s" ros2 run causal_slam point_cloud_rate_probe_node --ros-args \
    -r __node:="checked_rate_probe_${RUN_ID}" \
    -p topic:="${CHECKED_LIDAR_TOPIC}" \
    -p label:=checked_lidar \
    -p summary_period_ms:=2000.0 \
    -p qos_reliability:="${QOS_RELIABILITY}" \
    -p qos_depth:="${QOS_DEPTH}" >"${OUTPUT_HZ_LOG}" 2>&1 &
  HZ_OUTPUT_PID="$!"
  PIDS+=("${HZ_OUTPUT_PID}")

  setsid timeout "${DURATION_SEC}s" ros2 topic echo --full-length \
    "${DECISION_JSON_TOPIC}" >"${DECISION_JSON_LOG}" 2>&1 &
  DECISION_JSON_PID="$!"
  PIDS+=("${DECISION_JSON_PID}")
fi

wait "${HZ_INPUT_PID}" >/dev/null 2>&1 || true

if ! truthy "${PUBLISHER_ONLY}"; then
  wait "${HZ_OUTPUT_PID}" >/dev/null 2>&1 || true
  wait "${DECISION_JSON_PID}" >/dev/null 2>&1 || true
fi

echo
echo "== input LiDAR probe =="
tail -20 "${INPUT_HZ_LOG}" || true

if ! truthy "${PUBLISHER_ONLY}"; then
  echo
  echo "== checked LiDAR probe =="
  tail -20 "${OUTPUT_HZ_LOG}" || true

  echo
  echo "== decision JSON sample =="
  tail -20 "${DECISION_JSON_LOG}" || true

  echo
  echo "== monitor warnings =="
  grep -E "${MONITOR_BAD_RE}" "${MONITOR_LOG}" | tail -40 || true
fi

echo
echo "== fake LiDAR publisher metrics =="
grep "FakeLidarPublisher metrics" "${LIDAR_LOG}" | tail -20 || true

echo
echo "== validations =="

INPUT_TOTAL_HZ="$(extract_last_total_hz "${INPUT_HZ_LOG}" "input_lidar")"
check_float_gte "${INPUT_TOTAL_HZ}" "${MIN_INPUT_HZ}" "input_lidar total_hz"

if ! truthy "${PUBLISHER_ONLY}"; then
  CHECKED_TOTAL_HZ="$(extract_last_total_hz "${OUTPUT_HZ_LOG}" "checked_lidar")"
  check_float_gte "${CHECKED_TOTAL_HZ}" "${MIN_CHECKED_HZ}" "checked_lidar total_hz"

  if ! grep -q "data:" "${DECISION_JSON_LOG}"; then
    echo "ERROR: decision JSON log contains no messages"
    exit 1
  fi
  echo "OK: decision JSON messages observed"

  if grep -E "imu_window_empty|imu_window_incomplete|imu_window_missing_suffix" \
      "${DECISION_JSON_LOG}"; then
    echo "ERROR: false IMU window fault detected in decision JSON"
    exit 1
  fi
  echo "OK: no false IMU window faults in decision JSON"

  if truthy "${FAIL_ON_MONITOR_WARNINGS}"; then
    if grep -E "${MONITOR_BAD_RE}" "${MONITOR_LOG}"; then
      echo "ERROR: monitor warnings/errors detected"
      exit 1
    fi
    echo "OK: no monitor warnings/errors"
  fi
fi

echo
echo "runtime stress logs:"
if ! truthy "${PUBLISHER_ONLY}"; then
  echo "  ${MONITOR_LOG}"
  echo "  ${IMU_LOG}"
fi
echo "  ${LIDAR_LOG}"
echo "  ${INPUT_HZ_LOG}"
if ! truthy "${PUBLISHER_ONLY}"; then
  echo "  ${OUTPUT_HZ_LOG}"
  echo "  ${DECISION_JSON_LOG}"
fi

echo
echo "runtime stress result: OK"