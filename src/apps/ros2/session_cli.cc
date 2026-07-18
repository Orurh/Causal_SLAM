#include "apps/ros2/session_cli.h"

#include <cstdlib>
#include <ostream>
#include <string>
#include <string_view>

namespace causal_slam::apps::ros2 {
namespace {

bool IsHelpArg(std::string_view arg) {
  return arg == "--help" || arg == "-h";
}

std::optional<std::string> RequireValue(int argc, char** argv, int index, std::string_view option, std::ostream& err) {
  const int value_index = index + 1;
  if (value_index >= argc) {
    err << "Missing value for " << option << ".\n";
    return std::nullopt;
  }

  const std::string_view value{argv[value_index]};
  if (value.empty() || (value.size() >= 2 && value[0] == '-' && value[1] == '-')) {
    err << "Missing value for " << option << ".\n";
    return std::nullopt;
  }

  return std::string{value};
}

bool IsValidGateMode(const std::string& mode) {
  return mode == "observe" || mode == "drop_invalid" || mode == "drop_degraded" || mode == "strict";
}

std::optional<double> ParseDouble(const std::string& value, std::string_view option, std::ostream& err) {
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0') {
    err << "Invalid numeric value for " << option << ": " << value << ".\n";
    return std::nullopt;
  }
  return parsed;
}

}  // namespace

void PrintSessionUsage(std::ostream& out) {
  out << "Usage:\n"
      << "  causal_slam_session discover --bag <path>\n"
      << "\n"
      << "  causal_slam_session init \\\n"
      << "    --bag <path> \\\n"
      << "    --lidar-topic <topic> \\\n"
      << "    --imu-topic <topic> \\\n"
      << "    --output-dir <path> \\\n"
      << "    [--mode observe|drop_invalid|drop_degraded|strict] \\\n"
      << "    [--checked-lidar-topic <topic>] \\\n"
      << "    [--play-rate <rate>]\n"
      << "\n"
      << "  causal_slam_session summarize \\\n"
      << "    --diagnostics-bag <path> \\\n"
      << "    --output-dir <path>\n"
      << "\n"
      << "Commands:\n"
      << "  discover              Inspect rosbag2 topics and classify LiDAR/IMU candidates\n"
      << "  init                  Generate session.yaml, temporal_monitor.generated.yaml and run scripts\n"
      << "  summarize             Build a small report from recorded Causal-SLAM diagnostic topics\n"
      << "\n"
      << "Options:\n"
      << "  --bag <path>                    Path to rosbag2 dataset directory\n"
      << "  --diagnostics-bag <path>        Path to recorded runtime diagnostics bag\n"
      << "  --lidar-topic <topic>           LiDAR PointCloud2 topic\n"
      << "  --imu-topic <topic>             IMU topic\n"
      << "  --output-dir <path>             Directory for generated session artifacts\n"
      << "  --mode <mode>                   Gate mode, default: observe\n"
      << "  --checked-lidar-topic <topic>\n"
      << "                                  Output checked PointCloud2 topic, default: /causal_slam/checked_lidar\n"
      << "  --play-rate <rate>              rosbag playback rate, default: 0.5\n"
      << "  -h, --help                      Show this help message\n";
}

std::optional<SessionOptions> ParseSessionArgs(int argc, char** argv, std::ostream& err) {
  SessionOptions options;

  if (argc <= 1) {
    options.command = SessionCommand::kHelp;
    return options;
  }

  const std::string_view command{argv[1]};
  if (IsHelpArg(command)) {
    options.command = SessionCommand::kHelp;
    return options;
  }

  if (command == "discover") {
    options.command = SessionCommand::kDiscover;
  } else if (command == "init") {
    options.command = SessionCommand::kInit;
  } else if (command == "summarize") {
    options.command = SessionCommand::kSummarize;
  } else {
    err << "Unknown command: " << command << ".\n";
    return std::nullopt;
  }

  for (int i = 2; i < argc; ++i) {
    const std::string_view arg{argv[i]};

    if (IsHelpArg(arg)) {
      options.command = SessionCommand::kHelp;
      return options;
    }

    const auto value = RequireValue(argc, argv, i, arg, err);
    if (!value.has_value()) {
      return std::nullopt;
    }

    if (arg == "--bag") {
      options.bag_path = *value;
    } else if (arg == "--diagnostics-bag") {
      options.diagnostics_bag_path = *value;
    } else if (arg == "--lidar-topic") {
      options.lidar_topic = *value;
    } else if (arg == "--imu-topic") {
      options.imu_topic = *value;
    } else if (arg == "--checked-lidar-topic") {
      options.checked_lidar_topic = *value;
    } else if (arg == "--mode") {
      options.mode = *value;
    } else if (arg == "--output-dir") {
      options.output_dir = *value;
    } else if (arg == "--play-rate") {
      const auto parsed = ParseDouble(*value, arg, err);
      if (!parsed.has_value()) {
        return std::nullopt;
      }
      options.play_rate = *parsed;
    } else {
      err << "Unknown argument: " << arg << ".\n";
      return std::nullopt;
    }

    ++i;
  }

  if (options.command == SessionCommand::kDiscover) {
    if (options.bag_path.empty()) {
      err << "Missing required argument: --bag <path>.\n";
      return std::nullopt;
    }
    return options;
  }

  if (options.command == SessionCommand::kSummarize) {
    if (options.diagnostics_bag_path.empty()) {
      err << "Missing required argument: --diagnostics-bag <path>.\n";
      return std::nullopt;
    }
    if (options.output_dir.empty()) {
      err << "Missing required argument: --output-dir <path>.\n";
      return std::nullopt;
    }
    return options;
  }

  if (options.bag_path.empty()) {
    err << "Missing required argument: --bag <path>.\n";
    return std::nullopt;
  }

  if (options.lidar_topic.empty()) {
    err << "Missing required argument: --lidar-topic <topic>.\n";
    return std::nullopt;
  }

  if (options.imu_topic.empty()) {
    err << "Missing required argument: --imu-topic <topic>.\n";
    return std::nullopt;
  }

  if (options.output_dir.empty()) {
    err << "Missing required argument: --output-dir <path>.\n";
    return std::nullopt;
  }

  if (!IsValidGateMode(options.mode)) {
    err << "Invalid --mode: " << options.mode << ". Expected observe, drop_invalid, drop_degraded, or strict.\n";
    return std::nullopt;
  }

  if (options.play_rate <= 0.0) {
    err << "--play-rate must be positive.\n";
    return std::nullopt;
  }

  return options;
}

}  // namespace causal_slam::apps::ros2
