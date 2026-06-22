#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud/point_cloud2_time_field_extractor.h"
#include "ros_adapters/point_cloud2_field_inspector.h"

namespace causal_slam::ros_adapters {

using PointCloud2TimeFieldUnit = causal_slam::pointcloud::PointCloud2TimeFieldUnit;
using PointCloud2TimeFieldExtraction =
    causal_slam::pointcloud::PointCloud2TimeFieldExtraction;

[[nodiscard]] const char* ToString(PointCloud2TimeFieldUnit unit);

class PointCloud2TimeFieldExtractor final {
 public:
  [[nodiscard]] PointCloud2TimeFieldExtraction Extract(
      const sensor_msgs::msg::PointCloud2& cloud,
      const PointCloud2FieldInfo& time_field) const;
};

}  // namespace causal_slam::ros_adapters
