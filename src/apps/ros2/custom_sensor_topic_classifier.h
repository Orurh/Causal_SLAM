#pragma once

#include <cstdint>
#include <string_view>

namespace causal_slam::apps::ros2 {

enum class CustomSensorTopicClassification : std::uint8_t {
  kCustomLidar,
  kRawLidarPackets,
  kRawImuPackets,
  kRawSensorPackets,
  kNotCustomSensorType,
};

[[nodiscard]] bool IsKnownCustomSensorType(std::string_view topic_type);

[[nodiscard]] CustomSensorTopicClassification ClassifyCustomSensorTopic(std::string_view topic_name, std::string_view topic_type);

[[nodiscard]] const char* ToString(CustomSensorTopicClassification classification);

}  // namespace causal_slam::apps::ros2
