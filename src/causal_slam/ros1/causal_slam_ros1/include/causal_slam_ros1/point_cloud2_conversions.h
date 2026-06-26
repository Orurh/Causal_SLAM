#pragma once

#include <cstdint>
#include <vector>

#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

#include "domain/sensors/pointcloud/point_cloud2_field_inspector.h"
#include "domain/sensors/pointcloud/point_cloud2_time_field_extractor.h"

namespace causal_slam_ros1 {

[[nodiscard]] std::int64_t StampToNanoseconds(const ros::Time& stamp);

[[nodiscard]] std::vector<causal_slam::pointcloud::PointCloud2FieldInfo>
ToPointCloud2FieldInfos(const sensor_msgs::PointCloud2& cloud);

[[nodiscard]] causal_slam::pointcloud::PointCloud2CloudView
ToPointCloud2CloudView(const sensor_msgs::PointCloud2& cloud);

}  // namespace causal_slam_ros1
