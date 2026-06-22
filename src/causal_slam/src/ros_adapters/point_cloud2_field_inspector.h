#pragma once

#include <cstdint>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud/point_cloud2_field_inspector.h"

namespace causal_slam::ros_adapters {

using PointCloud2TimeFieldRole = causal_slam::pointcloud::PointCloud2TimeFieldRole;
using PointCloud2FieldInfo = causal_slam::pointcloud::PointCloud2FieldInfo;
using PointCloud2FieldInspection =
    causal_slam::pointcloud::PointCloud2FieldInspection;

[[nodiscard]] const char* ToString(PointCloud2TimeFieldRole role);
[[nodiscard]] const char* PointCloud2DatatypeToString(std::uint8_t datatype);

class PointCloud2FieldInspector final {
 public:
  [[nodiscard]] PointCloud2FieldInspection Inspect(
      const sensor_msgs::msg::PointCloud2& cloud) const;
};

}  // namespace causal_slam::ros_adapters
