#include "apps/ros2/point_cloud2_topic_classifier.h"

#include <algorithm>
#include <cctype>
#include <string>

namespace causal_slam::apps::ros2 {
namespace {

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

PointCloud2TopicClassification ClassifyPointCloud2TopicName(std::string_view topic_name) {
  const std::string normalized_name = LowercaseAscii(topic_name);

  if (Contains(normalized_name, "radar") || Contains(normalized_name, "camera") || Contains(normalized_name, "depth") ||
      Contains(normalized_name, "stereo") || Contains(normalized_name, "rgbd")) {
    return PointCloud2TopicClassification::kSuspiciousNonLidarName;
  }

  if (Contains(normalized_name, "filtered") || Contains(normalized_name, "deskew") || Contains(normalized_name, "registered") ||
      Contains(normalized_name, "submap") || Contains(normalized_name, "accumulated") || Contains(normalized_name, "fused") ||
      Contains(normalized_name, "cropped") || Contains(normalized_name, "ground") || Contains(normalized_name, "obstacle")) {
    return PointCloud2TopicClassification::kProcessedCloud;
  }

  if (Contains(normalized_name, "lidar") || Contains(normalized_name, "velodyne") || Contains(normalized_name, "ouster") ||
      Contains(normalized_name, "hesai") || Contains(normalized_name, "pandar") || Contains(normalized_name, "livox") ||
      Contains(normalized_name, "os_cloud_node") || Contains(normalized_name, "pointcloud") || Contains(normalized_name, "/points") ||
      Contains(normalized_name, "points_")) {
    return PointCloud2TopicClassification::kLikelyLidar;
  }

  return PointCloud2TopicClassification::kUnclassified;
}

const char* ToString(PointCloud2TopicClassification classification) {
  switch (classification) {
    case PointCloud2TopicClassification::kLikelyLidar:
      return "likely_lidar";
    case PointCloud2TopicClassification::kProcessedCloud:
      return "processed_cloud";
    case PointCloud2TopicClassification::kSuspiciousNonLidarName:
      return "suspicious_non_lidar_name";
    case PointCloud2TopicClassification::kUnclassified:
      return "unclassified_pointcloud2";
  }

  return "unclassified_pointcloud2";
}

}  // namespace causal_slam::apps::ros2
