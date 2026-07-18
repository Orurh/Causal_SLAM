#pragma once

#include <cstdint>
#include <string_view>

namespace causal_slam::apps::ros2 {

enum class PointCloud2TopicClassification : std::uint8_t {
  kLikelyLidar,
  kProcessedCloud,
  kSuspiciousNonLidarName,
  kUnclassified,
};

[[nodiscard]] PointCloud2TopicClassification ClassifyPointCloud2TopicName(std::string_view topic_name);

[[nodiscard]] const char* ToString(PointCloud2TopicClassification classification);

}  // namespace causal_slam::apps::ros2
