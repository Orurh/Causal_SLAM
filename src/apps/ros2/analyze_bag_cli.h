#pragma once

#include <iosfwd>
#include <optional>
#include <string>

namespace causal_slam::apps::ros2 {

struct AnalyzeBagOptions {
  bool show_help = false;
  bool list_topics = false;
  std::string bag_path;
  std::string lidar_topic;
  std::string imu_topic;
  std::string report_path;
  std::string html_report_path;
};

void PrintUsage(std::ostream& out);

[[nodiscard]] std::optional<AnalyzeBagOptions> ParseAnalyzeBagArgs(int argc, char** argv, std::ostream& err);

}  // namespace causal_slam::apps::ros2