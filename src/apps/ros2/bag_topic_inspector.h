#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace causal_slam::apps::ros2 {

struct BagTopicInfo {
  std::string name;
  std::string type;
  std::uint64_t message_count = 0;
};

struct BagTopicInspection {
  std::string bag_path;
  std::vector<BagTopicInfo> topics;
};

[[nodiscard]] BagTopicInspection InspectBagTopics(const std::string& bag_path);

}  // namespace causal_slam::apps::ros2
