#include "apps/ros2/analyze_bag_cli.h"

#include <ostream>
#include <string>
#include <string_view>

namespace causal_slam::apps::ros2 {
namespace {

bool IsHelpArg(std::string_view arg) {
  return arg == "--help" || arg == "-h";
}

bool IsValueOptionArg(std::string_view arg) {
  return arg == "--bag" || arg == "--lidar-topic" || arg == "--imu-topic" || arg == "--report" || arg == "--html-report";
}

bool IsFlagArg(std::string_view arg) {
  return arg == "--list-topics";
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

}  // namespace

void PrintUsage(std::ostream& out) {
  out << "Usage:\n"
      << "  causal_slam_analyze_bag \\\n"
      << "    --bag <path> \\\n"
      << "    --lidar-topic <topic> \\\n"
      << "    --imu-topic <topic> \\\n"
      << "    --report <path> \\\n"
      << "    [--html-report <path>]\n"
      << "\n"
      << "  causal_slam_analyze_bag \\\n"
      << "    --bag <path> \\\n"
      << "    --list-topics\n"
      << "\n"
      << "Options:\n"
      << "  --bag <path>           Path to rosbag2 directory\n"
      << "  --lidar-topic <topic>  LiDAR PointCloud2 topic\n"
      << "  --imu-topic <topic>    IMU topic\n"
      << "  --report <path>        Output JSON report path\n"
      << "  --html-report <path>   Output HTML report path\n"
      << "  --list-topics          List bag topics and message counts\n"
      << "  -h, --help             Show this help message\n";
}

std::optional<AnalyzeBagOptions> ParseAnalyzeBagArgs(int argc, char** argv, std::ostream& err) {
  AnalyzeBagOptions options;

  if (argc == 2 && IsHelpArg(argv[1])) {
    options.show_help = true;
    return options;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};

    if (IsHelpArg(arg)) {
      options.show_help = true;
      return options;
    }

    if (IsFlagArg(arg)) {
      if (arg == "--list-topics") {
        options.list_topics = true;
      }
      continue;
    }

    if (!IsValueOptionArg(arg)) {
      err << "Unknown argument: " << arg << ".\n";
      return std::nullopt;
    }

    const auto value = RequireValue(argc, argv, i, arg, err);
    if (!value.has_value()) {
      return std::nullopt;
    }

    if (arg == "--bag") {
      options.bag_path = *value;
    } else if (arg == "--lidar-topic") {
      options.lidar_topic = *value;
    } else if (arg == "--imu-topic") {
      options.imu_topic = *value;
    } else if (arg == "--report") {
      options.report_path = *value;
    } else if (arg == "--html-report") {
      options.html_report_path = *value;
    }

    ++i;
  }

  bool valid = true;
  if (options.bag_path.empty()) {
    err << "Missing required argument: --bag <path>.\n";
    valid = false;
  }

  if (options.list_topics) {
    return valid ? std::optional<AnalyzeBagOptions>{options} : std::nullopt;
  }

  if (options.lidar_topic.empty()) {
    err << "Missing required argument: --lidar-topic <topic>.\n";
    valid = false;
  }
  if (options.imu_topic.empty()) {
    err << "Missing required argument: --imu-topic <topic>.\n";
    valid = false;
  }
  if (options.report_path.empty()) {
    err << "Missing required argument: --report <path>.\n";
    valid = false;
  }

  if (!valid) {
    return std::nullopt;
  }

  return options;
}

}  // namespace causal_slam::apps::ros2