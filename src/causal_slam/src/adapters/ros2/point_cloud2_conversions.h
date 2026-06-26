#pragma once

#include <cstdint>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "domain/sensors/pointcloud/point_cloud2_field_inspector.h"
#include "domain/sensors/pointcloud/point_cloud2_time_field_extractor.h"

namespace causal_slam::ros_adapters {

[[nodiscard]] std::int64_t HeaderStampToNanoseconds(
    const builtin_interfaces::msg::Time& stamp);

[[nodiscard]] std::vector<causal_slam::pointcloud::PointCloud2FieldInfo>
ToPointCloud2FieldInfos(const sensor_msgs::msg::PointCloud2& cloud);

[[nodiscard]] causal_slam::pointcloud::PointCloud2CloudView
ToPointCloud2CloudView(const sensor_msgs::msg::PointCloud2& cloud);

}  // namespace causal_slam::ros_adapters
