#include "apps/ros2/custom_sensor_topic_classifier.h"

#include <algorithm>
#include <cctype>
#include <string>

namespace causal_slam::apps::ros2 {
namespace {

constexpr std::string_view kLivoxRosDriverType = "livox_ros_driver/msg/CustomMsg";
constexpr std::string_view kLivoxRosDriver2Type = "livox_ros_driver2/msg/CustomMsg";
constexpr std::string_view kOusterPacketType = "ouster_ros/msg/PacketMsg";

std::string LowercaseAscii(std::string_view value) {
  std::string normalized{value};

  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });

  return normalized;
}

bool Contains(std::string_view value, std::string_view fragment) {
  return value.find(fragment) != std::string_view::npos;
}

}  // namespace

bool IsKnownCustomSensorType(std::string_view topic_type) {
  return topic_type == kLivoxRosDriverType || topic_type == kLivoxRosDriver2Type || topic_type == kOusterPacketType;
}

CustomSensorTopicClassification ClassifyCustomSensorTopic(std::string_view topic_name, std::string_view topic_type) {
  if (topic_type == kLivoxRosDriverType || topic_type == kLivoxRosDriver2Type) {
    return CustomSensorTopicClassification::kCustomLidar;
  }

  if (topic_type != kOusterPacketType) {
    return CustomSensorTopicClassification::kNotCustomSensorType;
  }

  const std::string normalized_name = LowercaseAscii(topic_name);

  if (Contains(normalized_name, "imu")) {
    return CustomSensorTopicClassification::kRawImuPackets;
  }

  if (Contains(normalized_name, "lidar") || Contains(normalized_name, "point") || Contains(normalized_name, "cloud")) {
    return CustomSensorTopicClassification::kRawLidarPackets;
  }

  return CustomSensorTopicClassification::kRawSensorPackets;
}

const char* ToString(CustomSensorTopicClassification classification) {
  switch (classification) {
    case CustomSensorTopicClassification::kCustomLidar:
      return "custom_lidar";
    case CustomSensorTopicClassification::kRawLidarPackets:
      return "raw_lidar_packets";
    case CustomSensorTopicClassification::kRawImuPackets:
      return "raw_imu_packets";
    case CustomSensorTopicClassification::kRawSensorPackets:
      return "raw_sensor_packets";
    case CustomSensorTopicClassification::kNotCustomSensorType:
      return "not_custom_sensor_type";
  }

  return "not_custom_sensor_type";
}

}  // namespace causal_slam::apps::ros2
