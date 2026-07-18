#pragma once

#include <iosfwd>
#include <optional>
#include <string>

namespace causal_slam::apps::ros2 {

enum class SessionCommand {
  kHelp,
  kDiscover,
  kInit,
  kSummarize,
};

struct SessionOptions {
  SessionCommand command = SessionCommand::kHelp;

  std::string bag_path;
  std::string diagnostics_bag_path;
  std::string lidar_topic;
  std::string imu_topic;
  std::string checked_lidar_topic = "/causal_slam/checked_lidar";
  std::string mode = "observe";
  std::string output_dir;
  double play_rate = 0.5;
};

void PrintSessionUsage(std::ostream& out);

[[nodiscard]] std::optional<SessionOptions> ParseSessionArgs(int argc, char** argv, std::ostream& err);

}  // namespace causal_slam::apps::ros2
