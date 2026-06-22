#include "ros_adapters/point_cloud2_time_field_extractor.h"

#include "ros_adapters/point_cloud2_conversions.h"

namespace causal_slam::ros_adapters {

const char* ToString(PointCloud2TimeFieldUnit unit) {
  return causal_slam::pointcloud::ToString(unit);
}

PointCloud2TimeFieldExtraction PointCloud2TimeFieldExtractor::Extract(
    const sensor_msgs::msg::PointCloud2& cloud,
    const PointCloud2FieldInfo& time_field) const {
  const causal_slam::pointcloud::PointCloud2TimeFieldExtractor extractor;
  return extractor.Extract(ToPointCloud2CloudView(cloud), time_field);
}

}  // namespace causal_slam::ros_adapters
