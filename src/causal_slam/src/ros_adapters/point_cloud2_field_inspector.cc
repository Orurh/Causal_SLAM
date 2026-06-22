#include "ros_adapters/point_cloud2_field_inspector.h"

#include "ros_adapters/point_cloud2_conversions.h"

namespace causal_slam::ros_adapters {

const char* ToString(PointCloud2TimeFieldRole role) {
  return causal_slam::pointcloud::ToString(role);
}

const char* PointCloud2DatatypeToString(std::uint8_t datatype) {
  return causal_slam::pointcloud::PointCloud2DatatypeToString(datatype);
}

PointCloud2FieldInspection PointCloud2FieldInspector::Inspect(
    const sensor_msgs::msg::PointCloud2& cloud) const {
  const causal_slam::pointcloud::PointCloud2FieldInspector inspector;
  return inspector.Inspect(ToPointCloud2FieldInfos(cloud));
}

}  // namespace causal_slam::ros_adapters
