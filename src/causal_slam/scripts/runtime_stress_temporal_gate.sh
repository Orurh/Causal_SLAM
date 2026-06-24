#!/usr/bin/env bash
set -Eeuo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"

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

start_process() {
  local name="$1"
  shift

  local log_file="${LOG_DIR}/${RUN_ID}_${name}.log"
  echo "starting ${name}, log=${log_file}"

  setsid "$@" >"${log_file}" 2>&1 &
  PIDS+=("$!")
}

truthy() {
  case "$1" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
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
echo "lidar_topic=${LIDAR_TOPIC}"
echo "checked_lidar_topic=${CHECKED_LIDAR_TOPIC}"
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

INPUT_HZ_LOG="${LOG_DIR}/${RUN_ID}_input_hz.log"
OUTPUT_HZ_LOG="${LOG_DIR}/${RUN_ID}_output_hz.log"

setsid timeout "${DURATION_SEC}s" ros2 run causal_slam point_cloud_rate_probe_node --ros-args \
  -r __node:="input_rate_probe_${RUN_ID}" \
  -p topic:="${LIDAR_TOPIC}" \
  -p label:=input_lidar \
  -p summary_period_ms:=2000.0 \
  -p qos_reliability:="${QOS_RELIABILITY}" \
  -p qos_depth:="${QOS_DEPTH}" >"${INPUT_HZ_LOG}" 2>&1 &
HZ_INPUT_PID="$!"

if ! truthy "${PUBLISHER_ONLY}"; then
  setsid timeout "${DURATION_SEC}s" ros2 run causal_slam point_cloud_rate_probe_node --ros-args \
    -r __node:="checked_rate_probe_${RUN_ID}" \
    -p topic:="${CHECKED_LIDAR_TOPIC}" \
    -p label:=checked_lidar \
    -p summary_period_ms:=2000.0 \
    -p qos_reliability:="${QOS_RELIABILITY}" \
    -p qos_depth:="${QOS_DEPTH}" >"${OUTPUT_HZ_LOG}" 2>&1 &
  HZ_OUTPUT_PID="$!"
fi

wait "${HZ_INPUT_PID}" >/dev/null 2>&1 || true

if ! truthy "${PUBLISHER_ONLY}"; then
  wait "${HZ_OUTPUT_PID}" >/dev/null 2>&1 || true
fi

echo
echo "== input LiDAR probe =="
tail -20 "${INPUT_HZ_LOG}" || true

if ! truthy "${PUBLISHER_ONLY}"; then
  echo
  echo "== checked LiDAR probe =="
  tail -20 "${OUTPUT_HZ_LOG}" || true

  echo
  echo "== monitor warnings =="
  grep -E "WARN|ERROR|blocked|dropped" "${LOG_DIR}/${RUN_ID}_monitor.log" | tail -40 || true
fi

echo
echo "== fake LiDAR publisher metrics =="
grep "FakeLidarPublisher metrics" "${LOG_DIR}/${RUN_ID}_lidar.log" | tail -20 || true

echo
echo "runtime stress logs:"
if ! truthy "${PUBLISHER_ONLY}"; then
  echo "  ${LOG_DIR}/${RUN_ID}_monitor.log"
  echo "  ${LOG_DIR}/${RUN_ID}_imu.log"
fi
echo "  ${LOG_DIR}/${RUN_ID}_lidar.log"
echo "  ${INPUT_HZ_LOG}"
if ! truthy "${PUBLISHER_ONLY}"; then
  echo "  ${OUTPUT_HZ_LOG}"
fi
