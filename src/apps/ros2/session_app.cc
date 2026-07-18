#include "apps/ros2/session_app.h"

#include "apps/ros2/bag_topic_inspector.h"
#include "apps/ros2/custom_sensor_topic_classifier.h"
#include "apps/ros2/point_cloud2_topic_classifier.h"
#include "apps/ros2/session_cli.h"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <ostream>
#include <string>
#include <string_view>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace causal_slam::apps::ros2 {
namespace {

constexpr std::string_view kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr std::string_view kImuType = "sensor_msgs/msg/Imu";

constexpr std::string_view kFaultReasonsTopic = "/causal_slam/fault_reasons";
constexpr std::string_view kMapUpdateAllowedTopic = "/causal_slam/map_update_allowed";
constexpr std::string_view kMapUpdateDecisionJsonTopic = "/causal_slam/map_update_decision_json";

struct RuntimeDiagnosticSummary {
  std::string diagnostics_bag_path;
  std::map<std::string, std::uint64_t> topic_counts;
  std::map<std::string, std::uint64_t> fault_reason_counts;
  std::uint64_t map_update_allowed_true = 0;
  std::uint64_t map_update_allowed_false = 0;
  std::uint64_t decision_json_count = 0;
};

void PrintTopic(const BagTopicInfo& topic, std::ostream& out) {
  out << "  " << topic.name << " " << topic.type << " messages=" << topic.message_count << "\n";
}

void PrintPointCloud2Candidate(const BagTopicInfo& topic, std::ostream& out) {
  out << "  " << topic.name << " " << topic.type << " messages=" << topic.message_count
      << " classification=" << ToString(ClassifyPointCloud2TopicName(topic.name)) << "\n";
}

void PrintCustomSensorCandidate(const BagTopicInfo& topic, std::ostream& out) {
  out << "  " << topic.name << " " << topic.type << " messages=" << topic.message_count
      << " classification=" << ToString(ClassifyCustomSensorTopic(topic.name, topic.type)) << " support=not_supported_yet"
      << "\n";
}

std::string JsonEscape(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());

  for (const char c : value) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += c;
        break;
    }
  }

  return escaped;
}

std::string CsvEscape(const std::string& value) {
  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped += c;
    }
  }
  escaped += "\"";
  return escaped;
}

void WriteJsonMap(std::ostream& out, const std::map<std::string, std::uint64_t>& values, const std::string& indent) {
  out << "{\n";

  std::size_t index = 0;
  for (const auto& [key, value] : values) {
    out << indent << "  \"" << JsonEscape(key) << "\": " << value;
    if (++index < values.size()) {
      out << ",";
    }
    out << "\n";
  }

  out << indent << "}";
}

int RunDiscover(const SessionOptions& options, std::ostream& out, std::ostream& err) {
  try {
    const auto inspection = InspectBagTopics(options.bag_path);

    out << "Causal-SLAM session discovery\n";
    out << "Bag: " << inspection.bag_path << "\n\n";

    out << "PointCloud2 candidates:\n";
    bool has_point_cloud2 = false;
    for (const auto& topic : inspection.topics) {
      if (topic.type == kPointCloud2Type) {
        PrintPointCloud2Candidate(topic, out);
        has_point_cloud2 = true;
      }
    }
    if (!has_point_cloud2) {
      out << "  none\n";
    }

    out << "\nCustom sensor candidates:\n";
    bool has_custom_sensor = false;
    for (const auto& topic : inspection.topics) {
      if (IsKnownCustomSensorType(topic.type)) {
        PrintCustomSensorCandidate(topic, out);
        has_custom_sensor = true;
      }
    }
    if (!has_custom_sensor) {
      out << "  none\n";
    }

    out << "\nIMU candidates:\n";
    bool has_imu = false;
    for (const auto& topic : inspection.topics) {
      if (topic.type == kImuType) {
        PrintTopic(topic, out);
        has_imu = true;
      }
    }
    if (!has_imu) {
      out << "  none\n";
    }

    out << "\nOther topics:\n";
    bool has_other = false;
    for (const auto& topic : inspection.topics) {
      if (topic.type != kPointCloud2Type && topic.type != kImuType && !IsKnownCustomSensorType(topic.type)) {
        PrintTopic(topic, out);
        has_other = true;
      }
    }
    if (!has_other) {
      out << "  none\n";
    }

    return 0;
  } catch (const std::exception& e) {
    err << "Failed to discover session topics.\n"
        << "  bag: " << options.bag_path << "\n"
        << "  error: " << e.what() << "\n";
    return 2;
  }
}

bool ValidateInitTopics(const SessionOptions& options, std::ostream& err) {
  BagTopicInspection inspection;

  try {
    inspection = InspectBagTopics(options.bag_path);
  } catch (const std::exception& e) {
    err << "Failed to inspect bag before session init.\n"
        << "  bag: " << options.bag_path << "\n"
        << "  error: " << e.what() << "\n";
    return false;
  }

  const BagTopicInfo* lidar_topic = nullptr;
  const BagTopicInfo* imu_topic = nullptr;

  for (const auto& topic : inspection.topics) {
    if (topic.name == options.lidar_topic) {
      lidar_topic = &topic;
    }

    if (topic.name == options.imu_topic) {
      imu_topic = &topic;
    }
  }

  if (lidar_topic == nullptr) {
    err << "LiDAR topic not found in bag.\n"
        << "  bag: " << options.bag_path << "\n"
        << "  lidar_topic: " << options.lidar_topic << "\n"
        << "Hint: run `causal_slam_session discover --bag <path>` first.\n";
    return false;
  }

  if (lidar_topic->type != kPointCloud2Type) {
    err << "LiDAR topic has wrong type.\n"
        << "  topic: " << lidar_topic->name << "\n"
        << "  actual_type: " << lidar_topic->type << "\n"
        << "  expected_type: " << kPointCloud2Type << "\n";
    return false;
  }

  if (imu_topic == nullptr) {
    err << "IMU topic not found in bag.\n"
        << "  bag: " << options.bag_path << "\n"
        << "  imu_topic: " << options.imu_topic << "\n"
        << "Hint: run `causal_slam_session discover --bag <path>` first.\n";
    return false;
  }

  if (imu_topic->type != kImuType) {
    err << "IMU topic has wrong type.\n"
        << "  topic: " << imu_topic->name << "\n"
        << "  actual_type: " << imu_topic->type << "\n"
        << "  expected_type: " << kImuType << "\n";
    return false;
  }

  return true;
}

bool WriteSessionYaml(const std::filesystem::path& path, const SessionOptions& options, std::ostream& err) {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to write session config: " << path << "\n";
    return false;
  }

  file << "session:\n"
       << "  name: causal_slam_session\n"
       << "  output_dir: " << options.output_dir << "\n"
       << "\n"
       << "source:\n"
       << "  type: bag\n"
       << "  bag_path: " << options.bag_path << "\n"
       << "  play_rate: " << options.play_rate << "\n"
       << "  loop: false\n"
       << "\n"
       << "topics:\n"
       << "  lidar: " << options.lidar_topic << "\n"
       << "  imu: " << options.imu_topic << "\n"
       << "  checked_lidar: " << options.checked_lidar_topic << "\n"
       << "\n"
       << "mode:\n"
       << "  gate: " << options.mode << "\n"
       << "\n"
       << "report:\n"
       << "  enabled: true\n"
       << "  record_clouds: false\n"
       << "  diagnostic_topics:\n"
       << "    - /causal_slam/fault_reasons\n"
       << "    - /causal_slam/map_update_allowed\n"
       << "    - /causal_slam/map_update_decision_json\n"
       << "\n"
       << "gui:\n"
       << "  foxglove_enabled: true\n"
       << "  foxglove_port: 8765\n";

  return true;
}

bool WriteTemporalMonitorYaml(const std::filesystem::path& path, const SessionOptions& options, std::ostream& err) {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to write temporal monitor config: " << path << "\n";
    return false;
  }

  file << "temporal_monitor_node:\n"
       << "  ros__parameters:\n"
       << "    runtime_profile: diagnostic\n"
       << "\n"
       << "    lidar_topic: " << options.lidar_topic << "\n"
       << "    imu_topic: " << options.imu_topic << "\n"
       << "    checked_lidar_topic: " << options.checked_lidar_topic << "\n"
       << "\n"
       << "    map_update_allowed_topic: /causal_slam/map_update_allowed\n"
       << "    temporal_health_topic: /causal_slam/temporal_health\n"
       << "    map_update_reason_topic: /causal_slam/map_update_reason\n"
       << "    fault_reasons_topic: /causal_slam/fault_reasons\n"
       << "    map_update_decision_json_topic: /causal_slam/map_update_decision_json\n"
       << "\n"
       << "    lidar_gate_mode: " << options.mode << "\n"
       << "\n"
       << "    gate_min_total_imu_samples_before_forward: 5\n"
       << "    gate_min_window_imu_samples_before_forward: 2\n"
       << "\n"
       << "    summary_period_ms: 2000.0\n"
       << "    html_report_path: \"\"\n"
       << "\n"
       << "    lidar_qos_reliability: best_effort\n"
       << "    lidar_qos_depth: 5\n"
       << "    checked_lidar_qos_reliability: reliable\n"
       << "    checked_lidar_qos_depth: 5\n"
       << "\n"
       << "    tf_monitoring_enabled: false\n"
       << "\n"
       << "    expected_imu_period_ms: 10.0\n"
       << "    imu_buffer_retention_ms: 5000.0\n"
       << "    imu_gap_threshold_ms: 30.0\n"
       << "    lidar_gap_threshold_ms: 200.0\n"
       << "\n"
       << "    lidar_scan_duration_ms: 100.0\n"
       << "    lidar_min_measured_scan_duration_ms: 1.0\n"
       << "    lidar_max_measured_scan_duration_ms: 500.0\n"
       << "    lidar_prefer_measured_header_period: false\n"
       << "    lidar_stamp_policy: scan_end\n"
       << "\n"
       << "    lidar_point_time_mode: auto\n"
       << "    lidar_point_time_field: \"\"\n"
       << "    lidar_point_time_interpretation: auto\n"
       << "    lidar_point_time_unit: auto\n"
       << "\n"
       << "    lidar_holdback_enabled: true\n"
       << "    lidar_holdback_tolerance_ms: 10.0\n"
       << "    lidar_holdback_max_pending: 64\n"
       << "\n"
       << "    max_missing_prefix_ms: 30.0\n"
       << "    max_missing_suffix_ms: 30.0\n"
       << "    max_internal_gap_ms: 30.0\n";

  return true;
}

bool WriteExecutableScript(const std::filesystem::path& path, const std::string& body, std::ostream& err) {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to write script: " << path << "\n";
    return false;
  }

  file << body;
  file.close();

  std::filesystem::permissions(
      path, std::filesystem::perms::owner_exec | std::filesystem::perms::group_exec | std::filesystem::perms::others_exec,
      std::filesystem::perm_options::add);
  return true;
}

bool WriteRunScripts(const std::filesystem::path& output_dir, const SessionOptions& options, std::ostream& err) {
  const auto run_temporal_monitor = output_dir / "run_temporal_monitor.sh";
  const auto run_bag_play = output_dir / "run_bag_play.sh";
  const auto record_diagnostics = output_dir / "record_diagnostics.sh";
  const auto summarize_diagnostics = output_dir / "summarize_diagnostics.sh";
  const auto run_all_observe = output_dir / "run_all_observe.sh";
  const auto run_foxglove_bridge = output_dir / "run_foxglove_bridge.sh";

  const std::string temporal_monitor_script =
      "#!/usr/bin/env bash\n"
      "set -eo pipefail\n"
      "\n"
      "export ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID:-42}\"\n"
      "source /opt/ros/jazzy/setup.bash\n"
      "if [[ -n \"${CAUSAL_SLAM_SETUP_FILE:-}\" ]]; then\n"
      "  source \"$CAUSAL_SLAM_SETUP_FILE\"\n"
      "elif [[ -f \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "elif [[ -f \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "fi\n"
      "set -u\n"
      "\n"
      "SCRIPT_DIR=\"$(cd \"$(dirname \"${BASH_SOURCE[0]}\")\" && pwd)\"\n"
      "\n"
      "ros2 run causal_slam temporal_monitor_node \\\n"
      "  --ros-args \\\n"
      "  --params-file \"$SCRIPT_DIR/temporal_monitor.generated.yaml\"\n";

  const std::string bag_play_script =
      "#!/usr/bin/env bash\n"
      "set -eo pipefail\n"
      "\n"
      "export ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID:-42}\"\n"
      "source /opt/ros/jazzy/setup.bash\n"
      "if [[ -n \"${CAUSAL_SLAM_SETUP_FILE:-}\" ]]; then\n"
      "  source \"$CAUSAL_SLAM_SETUP_FILE\"\n"
      "elif [[ -f \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "elif [[ -f \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "fi\n"
      "set -u\n"
      "\n"
      "ros2 bag play \"" +
      options.bag_path +
      "\" \\\n"
      "  --clock \\\n"
      "  -r " +
      std::to_string(options.play_rate) +
      " \\\n"
      "  --topics \\\n"
      "  " +
      options.lidar_topic +
      " \\\n"
      "  " +
      options.imu_topic + "\n";

  const std::string record_diagnostics_script =
      "#!/usr/bin/env bash\n"
      "set -eo pipefail\n"
      "\n"
      "export ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID:-42}\"\n"
      "source /opt/ros/jazzy/setup.bash\n"
      "if [[ -n \"${CAUSAL_SLAM_SETUP_FILE:-}\" ]]; then\n"
      "  source \"$CAUSAL_SLAM_SETUP_FILE\"\n"
      "elif [[ -f \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "elif [[ -f \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "fi\n"
      "set -u\n"
      "\n"
      "SCRIPT_DIR=\"$(cd \"$(dirname \"${BASH_SOURCE[0]}\")\" && pwd)\"\n"
      "REPORT_BAG=\"$SCRIPT_DIR/runtime_diagnostics_$(date +%Y%m%d_%H%M%S)\"\n"
      "\n"
      "ros2 bag record \\\n"
      "  -o \"$REPORT_BAG\" \\\n"
      "  --topics \\\n"
      "  /causal_slam/fault_reasons \\\n"
      "  /causal_slam/map_update_allowed \\\n"
      "  /causal_slam/map_update_decision_json\n";

  const std::string summarize_diagnostics_script =
      "#!/usr/bin/env bash\n"
      "set -eo pipefail\n"
      "\n"
      "if [[ $# -ne 1 ]]; then\n"
      "  echo \"usage: ./summarize_diagnostics.sh <runtime_diagnostics_bag_dir>\" >&2\n"
      "  exit 2\n"
      "fi\n"
      "\n"
      "source /opt/ros/jazzy/setup.bash\n"
      "if [[ -n \"${CAUSAL_SLAM_SETUP_FILE:-}\" ]]; then\n"
      "  source \"$CAUSAL_SLAM_SETUP_FILE\"\n"
      "elif [[ -f \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "elif [[ -f \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "fi\n"
      "set -u\n"
      "\n"
      "SCRIPT_DIR=\"$(cd \"$(dirname \"${BASH_SOURCE[0]}\")\" && pwd)\"\n"
      "\n"
      "ros2 run causal_slam causal_slam_session summarize \\\n"
      "  --diagnostics-bag \"$1\" \\\n"
      "  --output-dir \"$SCRIPT_DIR\"\n";

  const std::string run_all_observe_script = R"SH(#!/usr/bin/env bash
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MONITOR_PID=""
RECORDER_PID=""
PLAY_PID=""
CLEANUP_STARTED=0

STOP_INT_TIMEOUT_SEC="${CAUSAL_SLAM_STOP_INT_TIMEOUT_SEC:-1}"
STOP_TERM_TIMEOUT_SEC="${CAUSAL_SLAM_STOP_TERM_TIMEOUT_SEC:-3}"
STOP_KILL_TIMEOUT_SEC="${CAUSAL_SLAM_STOP_KILL_TIMEOUT_SEC:-2}"

process_group_has_live_processes() {
  local pgid="$1"

  if [ -z "$pgid" ]; then
    return 1
  fi

  LC_ALL=C ps -eo pgid=,stat= 2>/dev/null |
    awk -v pgid="$pgid" '
      $1 == pgid && $2 !~ /^Z/ {
        found = 1
      }
      END {
        exit(found ? 0 : 1)
      }
    '
}

wait_for_process_group_exit() {
  local pgid="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))

  while process_group_has_live_processes "$pgid"; do
    if (( SECONDS >= deadline )); then
      return 1
    fi

    sleep 0.1
  done

  return 0
}

stop_process_group() {
  local name="$1"
  local pgid="$2"

  if [ -z "$pgid" ]; then
    return 0
  fi

  if ! process_group_has_live_processes "$pgid"; then
    wait "$pgid" 2>/dev/null || true
    return 0
  fi

  echo "[causal_slam_session] Sending SIGINT to $name process group $pgid..."
  kill -s INT -- "-$pgid" 2>/dev/null || true

  if wait_for_process_group_exit "$pgid" "$STOP_INT_TIMEOUT_SEC"; then
    wait "$pgid" 2>/dev/null || true
    return 0
  fi

  echo "[causal_slam_session] $name did not stop after SIGINT; sending SIGTERM..." >&2
  kill -s TERM -- "-$pgid" 2>/dev/null || true

  if wait_for_process_group_exit "$pgid" "$STOP_TERM_TIMEOUT_SEC"; then
    wait "$pgid" 2>/dev/null || true
    return 0
  fi

  echo "[causal_slam_session] $name did not stop after SIGTERM; sending SIGKILL..." >&2
  kill -s KILL -- "-$pgid" 2>/dev/null || true
  wait_for_process_group_exit "$pgid" "$STOP_KILL_TIMEOUT_SEC" || true
  wait "$pgid" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?

  if [ "$CLEANUP_STARTED" -ne 0 ]; then
    return "$exit_code"
  fi

  CLEANUP_STARTED=1
  trap '' INT TERM

  stop_process_group "bag player" "$PLAY_PID"
  stop_process_group "diagnostics recorder" "$RECORDER_PID"
  stop_process_group "temporal monitor" "$MONITOR_PID"

  PLAY_PID=""
  RECORDER_PID=""
  MONITOR_PID=""

  return "$exit_code"
}

handle_signal() {
  local signal_name="$1"
  local exit_code=130

  if [ "$signal_name" = "TERM" ]; then
    exit_code=143
  fi

  echo
  echo "[causal_slam_session] Received SIG$signal_name; stopping child processes..." >&2
  cleanup
  exit "$exit_code"
}

trap cleanup EXIT
trap 'handle_signal INT' INT
trap 'handle_signal TERM' TERM

if ! command -v setsid >/dev/null 2>&1; then
  echo "[causal_slam_session] Required command not found: setsid" >&2
  exit 4
fi

echo "[causal_slam_session] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[causal_slam_session] Starting temporal monitor..."
setsid ./run_temporal_monitor.sh </dev/null &
MONITOR_PID="$!"

sleep "${CAUSAL_SLAM_MONITOR_STARTUP_DELAY_SEC:-2}"

echo "[causal_slam_session] Starting diagnostic recorder..."
setsid ./record_diagnostics.sh </dev/null &
RECORDER_PID="$!"

sleep "${CAUSAL_SLAM_RECORDER_STARTUP_DELAY_SEC:-2}"

echo "[causal_slam_session] Playing source bag..."
setsid ./run_bag_play.sh </dev/null &
PLAY_PID="$!"

if wait "$PLAY_PID"; then
  PLAY_EXIT=0
else
  PLAY_EXIT="$?"
fi

PLAY_PID=""

echo "[causal_slam_session] Stopping recorder and monitor..."
stop_process_group "diagnostics recorder" "$RECORDER_PID"
RECORDER_PID=""

stop_process_group "temporal monitor" "$MONITOR_PID"
MONITOR_PID=""

trap - EXIT INT TERM

if [ "$PLAY_EXIT" -ne 0 ]; then
  echo "[causal_slam_session] Bag playback failed with exit code $PLAY_EXIT" >&2
  exit "$PLAY_EXIT"
fi

DIAG_BAG="$(find "$SCRIPT_DIR" -maxdepth 1 -type d -name 'runtime_diagnostics_*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"

if [ -z "$DIAG_BAG" ]; then
  echo "[causal_slam_session] No runtime_diagnostics_* bag was produced." >&2
  exit 3
fi

echo "[causal_slam_session] Summarizing diagnostics bag:"
echo "  $DIAG_BAG"
./summarize_diagnostics.sh "$DIAG_BAG"

echo
echo "[causal_slam_session] Done."
echo "Summary files:"
echo "  $SCRIPT_DIR/diagnostic_summary.txt"
echo "  $SCRIPT_DIR/diagnostic_summary.json"
echo "  $SCRIPT_DIR/diagnostic_timeline.csv"
echo "  $SCRIPT_DIR/map_update_decision_json.jsonl"
)SH";

  const std::string foxglove_script =
      "#!/usr/bin/env bash\n"
      "set -eo pipefail\n"
      "\n"
      "export ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID:-42}\"\n"
      "source /opt/ros/jazzy/setup.bash\n"
      "if [[ -n \"${CAUSAL_SLAM_SETUP_FILE:-}\" ]]; then\n"
      "  source \"$CAUSAL_SLAM_SETUP_FILE\"\n"
      "elif [[ -f \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Docs/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "elif [[ -f \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\" ]]; then\n"
      "  source \"$HOME/Документы/VSCode/causal_slam_ws/install/setup.bash\"\n"
      "fi\n"
      "set -u\n"
      "\n"
      "ros2 run foxglove_bridge foxglove_bridge \\\n"
      "  --ros-args \\\n"
      "  -p port:=8765 \\\n"
      "  -p address:=0.0.0.0\n";

  return WriteExecutableScript(run_temporal_monitor, temporal_monitor_script, err) &&
         WriteExecutableScript(run_bag_play, bag_play_script, err) &&
         WriteExecutableScript(record_diagnostics, record_diagnostics_script, err) &&
         WriteExecutableScript(summarize_diagnostics, summarize_diagnostics_script, err) &&
         WriteExecutableScript(run_all_observe, run_all_observe_script, err) &&
         WriteExecutableScript(run_foxglove_bridge, foxglove_script, err);
}

RuntimeDiagnosticSummary ReadRuntimeDiagnostics(const std::string& diagnostics_bag_path, const std::filesystem::path& output_dir) {
  RuntimeDiagnosticSummary summary;
  summary.diagnostics_bag_path = diagnostics_bag_path;

  rosbag2_cpp::Reader reader;
  reader.open(diagnostics_bag_path);

  rclcpp::Serialization<std_msgs::msg::String> string_serializer;
  rclcpp::Serialization<std_msgs::msg::Bool> bool_serializer;

  std::ofstream timeline{output_dir / "diagnostic_timeline.csv"};
  std::ofstream decisions{output_dir / "map_update_decision_json.jsonl"};

  timeline << "bag_time_ns,topic,value\n";

  while (reader.has_next()) {
    const auto message = reader.read_next();
    ++summary.topic_counts[message->topic_name];

    if (message->topic_name == kFaultReasonsTopic) {
      rclcpp::SerializedMessage serialized_message{*message->serialized_data};
      std_msgs::msg::String msg;
      string_serializer.deserialize_message(&serialized_message, &msg);

      ++summary.fault_reason_counts[msg.data];
      timeline << message->recv_timestamp << "," << CsvEscape(message->topic_name) << "," << CsvEscape(msg.data) << "\n";
      continue;
    }

    if (message->topic_name == kMapUpdateAllowedTopic) {
      rclcpp::SerializedMessage serialized_message{*message->serialized_data};
      std_msgs::msg::Bool msg;
      bool_serializer.deserialize_message(&serialized_message, &msg);

      if (msg.data) {
        ++summary.map_update_allowed_true;
        timeline << message->recv_timestamp << "," << CsvEscape(message->topic_name) << "," << CsvEscape("true") << "\n";
      } else {
        ++summary.map_update_allowed_false;
        timeline << message->recv_timestamp << "," << CsvEscape(message->topic_name) << "," << CsvEscape("false") << "\n";
      }
      continue;
    }

    if (message->topic_name == kMapUpdateDecisionJsonTopic) {
      rclcpp::SerializedMessage serialized_message{*message->serialized_data};
      std_msgs::msg::String msg;
      string_serializer.deserialize_message(&serialized_message, &msg);

      ++summary.decision_json_count;
      decisions << msg.data << "\n";
      timeline << message->recv_timestamp << "," << CsvEscape(message->topic_name) << "," << CsvEscape(msg.data) << "\n";
      continue;
    }
  }

  return summary;
}

bool WriteDiagnosticSummaryText(const std::filesystem::path& path, const RuntimeDiagnosticSummary& summary, std::ostream& err) {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to write diagnostic summary: " << path << "\n";
    return false;
  }

  file << "Causal-SLAM runtime diagnostic summary\n"
       << "=====================================\n\n"
       << "Diagnostics bag: " << summary.diagnostics_bag_path << "\n\n";

  file << "Topic counts:\n";
  for (const auto& [topic, count] : summary.topic_counts) {
    file << "  " << topic << ": " << count << "\n";
  }

  file << "\nFault reason counts:\n";
  if (summary.fault_reason_counts.empty()) {
    file << "  none\n";
  } else {
    for (const auto& [reason, count] : summary.fault_reason_counts) {
      file << "  " << reason << ": " << count << "\n";
    }
  }

  file << "\nMap update allowed counts:\n"
       << "  True: " << summary.map_update_allowed_true << "\n"
       << "  False: " << summary.map_update_allowed_false << "\n"
       << "\nDecision JSON messages: " << summary.decision_json_count << "\n";

  return true;
}

bool WriteDiagnosticSummaryJson(const std::filesystem::path& path, const RuntimeDiagnosticSummary& summary, std::ostream& err) {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to write diagnostic summary JSON: " << path << "\n";
    return false;
  }

  file << "{\n"
       << "  \"diagnostics_bag_path\": \"" << JsonEscape(summary.diagnostics_bag_path) << "\",\n"
       << "  \"topic_counts\": ";
  WriteJsonMap(file, summary.topic_counts, "  ");
  file << ",\n"
       << "  \"fault_reason_counts\": ";
  WriteJsonMap(file, summary.fault_reason_counts, "  ");
  file << ",\n"
       << "  \"map_update_allowed_counts\": {\n"
       << "    \"True\": " << summary.map_update_allowed_true << ",\n"
       << "    \"False\": " << summary.map_update_allowed_false << "\n"
       << "  },\n"
       << "  \"decision_json_count\": " << summary.decision_json_count << ",\n"
       << "  \"outputs\": {\n"
       << "    \"diagnostic_timeline_csv\": \"diagnostic_timeline.csv\",\n"
       << "    \"map_update_decision_json_jsonl\": \"map_update_decision_json.jsonl\"\n"
       << "  }\n"
       << "}\n";

  return true;
}

bool WriteGeneratedReadme(const std::filesystem::path& path, const SessionOptions& options, std::ostream& err) {
  std::ofstream file{path};
  if (!file) {
    err << "Failed to write generated README: " << path << "\n";
    return false;
  }

  file << "# Causal-SLAM generated session\n\n"
       << "This directory was generated by `causal_slam_session init`.\n\n"
       << "## Scope\n\n"
       << "This is a Causal-SLAM diagnostics/preflight session.\n\n"
       << "It starts the temporal monitor, plays selected LiDAR/IMU topics, records Causal-SLAM diagnostic topics, and builds a small "
          "runtime summary.\n\n"
       << "It does **not** run a SLAM estimator by itself. No `/cloud_registered`, `/odometry`, or `/path` topics are expected unless you "
          "start a SLAM/LIO node separately.\n\n"
       << "For SLAM evaluation, connect your estimator's LiDAR input to `" << options.checked_lidar_topic << "` and its IMU input to `"
       << options.imu_topic << "`.\n\n"
       << "## Dataset\n\n"
       << "```text\n"
       << "bag: " << options.bag_path << "\n"
       << "lidar_topic: " << options.lidar_topic << "\n"
       << "imu_topic: " << options.imu_topic << "\n"
       << "gate_mode: " << options.mode << "\n"
       << "checked_lidar_topic: " << options.checked_lidar_topic << "\n"
       << "play_rate: " << options.play_rate << "\n"
       << "```\n\n"
       << "## Quick run\n\n"
       << "Use the generated one-command observe workflow first.\n\n"
       << "```bash\n"
       << "cd " << options.output_dir << "\n"
       << "./run_all_observe.sh\n"
       << "```\n\n"
       << "## Manual run order\n\n"
       << "Use the same `ROS_DOMAIN_ID` in all terminals.\n\n"
       << "### Terminal 1: temporal monitor\n\n"
       << "```bash\n"
       << "cd " << options.output_dir << "\n"
       << "ROS_DOMAIN_ID=42 ./run_temporal_monitor.sh\n"
       << "```\n\n"
       << "### Terminal 2: diagnostics recorder\n\n"
       << "```bash\n"
       << "cd " << options.output_dir << "\n"
       << "ROS_DOMAIN_ID=42 ./record_diagnostics.sh\n"
       << "```\n\n"
       << "Stop this terminal with `Ctrl+C` after bag playback finishes.\n\n"
       << "### Terminal 3: bag playback\n\n"
       << "```bash\n"
       << "cd " << options.output_dir << "\n"
       << "ROS_DOMAIN_ID=42 ./run_bag_play.sh\n"
       << "```\n\n"
       << "## Summarize diagnostics\n\n"
       << "```bash\n"
       << "cd " << options.output_dir << "\n"
       << "DIAG_BAG=\"$(ls -dt runtime_diagnostics_* | head -1)\"\n"
       << "./summarize_diagnostics.sh \"$DIAG_BAG\"\n"
       << "sed -n '1,160p' diagnostic_summary.txt\n"
       << "jq '.topic_counts, .fault_reason_counts, .map_update_allowed_counts' diagnostic_summary.json\n"
       << "```\n\n"
       << "## Foxglove visualization\n\n"
       << "This session can visualize raw and checked point clouds, but it does not create a SLAM map.\n\n"
       << "Useful topics:\n\n"
       << "```text\n"
       << options.lidar_topic << "\n"
       << options.checked_lidar_topic << "\n"
       << "/causal_slam/fault_reasons\n"
       << "/causal_slam/map_update_allowed\n"
       << "/causal_slam/map_update_decision_json\n"
       << "```\n\n"
       << "If Foxglove does not show the cloud, check the cloud frame id:\n\n"
       << "```bash\n"
       << "source /opt/ros/jazzy/setup.bash\n"
       << "export ROS_DOMAIN_ID=42\n"
       << "ros2 topic echo " << options.lidar_topic << " --once --field header.frame_id\n"
       << "```\n\n"
       << "Then use that value as the 3D panel Fixed frame, or publish a temporary static transform to `map`.\n\n"
       << "## Optional SLAM hookup\n\n"
       << "Start your SLAM/LIO estimator separately and remap its LiDAR input to:\n\n"
       << "```text\n"
       << options.checked_lidar_topic << "\n"
       << "```\n\n"
       << "Keep the IMU input as:\n\n"
       << "```text\n"
       << options.imu_topic << "\n"
       << "```\n\n"
       << "Only then should you expect SLAM outputs such as `/cloud_registered`, `/odometry`, or `/path`.\n\n"
       << "## Generated files\n\n"
       << "- `session.yaml` — high-level session description\n"
       << "- `temporal_monitor.generated.yaml` — ROS2 parameters for `temporal_monitor_node`\n"
       << "- `run_temporal_monitor.sh` — starts Causal-SLAM temporal monitor\n"
       << "- `record_diagnostics.sh` — records only diagnostic topics\n"
       << "- `run_bag_play.sh` — plays the source rosbag2 dataset\n"
       << "- `summarize_diagnostics.sh` — builds text/JSON/CSV summaries from a diagnostics bag\n"
       << "- `run_all_observe.sh` — runs monitor, recorder, bag playback, shutdown, and summary in one command\n"
       << "- `run_foxglove_bridge.sh` — optional Foxglove bridge\n";

  return true;
}

int RunInit(const SessionOptions& options, std::ostream& out, std::ostream& err) {
  try {
    if (!ValidateInitTopics(options, err)) {
      return 2;
    }

    const std::filesystem::path output_dir{options.output_dir};
    std::filesystem::create_directories(output_dir);

    const auto session_yaml = output_dir / "session.yaml";
    const auto temporal_monitor_yaml = output_dir / "temporal_monitor.generated.yaml";
    const auto readme = output_dir / "README.generated.md";

    if (!WriteSessionYaml(session_yaml, options, err)) {
      return 3;
    }

    if (!WriteTemporalMonitorYaml(temporal_monitor_yaml, options, err)) {
      return 3;
    }

    if (!WriteRunScripts(output_dir, options, err)) {
      return 3;
    }

    if (!WriteGeneratedReadme(readme, options, err)) {
      return 3;
    }

    out << "Causal-SLAM session initialized\n"
        << "Output dir: " << output_dir << "\n"
        << "Session config: " << session_yaml << "\n"
        << "Temporal monitor config: " << temporal_monitor_yaml << "\n"
        << "README: " << readme << "\n"
        << "\n"
        << "Run scripts:\n"
        << "  " << output_dir / "run_temporal_monitor.sh" << "\n"
        << "  " << output_dir / "run_bag_play.sh" << "\n"
        << "  " << output_dir / "record_diagnostics.sh" << "\n"
        << "  " << output_dir / "summarize_diagnostics.sh" << "\n"
        << "  " << output_dir / "run_all_observe.sh" << "\n"
        << "  " << output_dir / "run_foxglove_bridge.sh" << "\n"
        << "\n"
        << "Next:\n"
        << "  cd " << output_dir << "\n"
        << "  ROS_DOMAIN_ID=42 ./run_all_observe.sh\n";

    return 0;
  } catch (const std::exception& e) {
    err << "Failed to initialize session.\n"
        << "  output_dir: " << options.output_dir << "\n"
        << "  error: " << e.what() << "\n";
    return 2;
  }
}

int RunSummarize(const SessionOptions& options, std::ostream& out, std::ostream& err) {
  try {
    const std::filesystem::path output_dir{options.output_dir};
    std::filesystem::create_directories(output_dir);

    const auto summary = ReadRuntimeDiagnostics(options.diagnostics_bag_path, output_dir);

    const auto summary_txt = output_dir / "diagnostic_summary.txt";
    const auto summary_json = output_dir / "diagnostic_summary.json";

    if (!WriteDiagnosticSummaryText(summary_txt, summary, err)) {
      return 3;
    }

    if (!WriteDiagnosticSummaryJson(summary_json, summary, err)) {
      return 3;
    }

    out << "Causal-SLAM runtime diagnostics summarized\n"
        << "Diagnostics bag: " << options.diagnostics_bag_path << "\n"
        << "Output dir: " << output_dir << "\n"
        << "Summary text: " << summary_txt << "\n"
        << "Summary JSON: " << summary_json << "\n"
        << "Timeline CSV: " << output_dir / "diagnostic_timeline.csv" << "\n"
        << "Decision JSONL: " << output_dir / "map_update_decision_json.jsonl" << "\n"
        << "\n"
        << "Map update allowed:\n"
        << "  True: " << summary.map_update_allowed_true << "\n"
        << "  False: " << summary.map_update_allowed_false << "\n";

    return 0;
  } catch (const std::exception& e) {
    err << "Failed to summarize runtime diagnostics.\n"
        << "  diagnostics_bag: " << options.diagnostics_bag_path << "\n"
        << "  output_dir: " << options.output_dir << "\n"
        << "  error: " << e.what() << "\n";
    return 2;
  }
}

}  // namespace

int RunSessionCli(int argc, char** argv, std::ostream& out, std::ostream& err) {
  const auto options = ParseSessionArgs(argc, argv, err);
  if (!options.has_value()) {
    return 2;
  }

  if (options->command == SessionCommand::kHelp) {
    PrintSessionUsage(out);
    return 0;
  }

  if (options->command == SessionCommand::kDiscover) {
    return RunDiscover(*options, out, err);
  }

  if (options->command == SessionCommand::kInit) {
    return RunInit(*options, out, err);
  }

  if (options->command == SessionCommand::kSummarize) {
    return RunSummarize(*options, out, err);
  }

  PrintSessionUsage(out);
  return 0;
}

}  // namespace causal_slam::apps::ros2
