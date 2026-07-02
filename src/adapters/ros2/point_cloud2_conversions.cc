#include "point_cloud2_conversions.h"

#include "domain/time/time_units.h"

namespace causal_slam::ros_adapters {
namespace {}  // namespace

std::int64_t HeaderStampToNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * static_cast<std::int64_t>(causal_slam::core::kNanosecondsPerSecond) +
         static_cast<std::int64_t>(stamp.nanosec);
}

std::vector<causal_slam::pointcloud::PointCloud2FieldInfo> ToPointCloud2FieldInfos(const sensor_msgs::msg::PointCloud2& cloud) {
  std::vector<causal_slam::pointcloud::PointCloud2FieldInfo> fields;
  fields.reserve(cloud.fields.size());

  for (const auto& field : cloud.fields) {
    fields.push_back(causal_slam::pointcloud::PointCloud2FieldInfo{
        .name = field.name,
        .offset = field.offset,
        .datatype = field.datatype,
        .count = field.count,
    });
  }

  return fields;
}

causal_slam::pointcloud::PointCloud2CloudView ToPointCloud2CloudView(const sensor_msgs::msg::PointCloud2& cloud) {
  return causal_slam::pointcloud::PointCloud2CloudView{
      .header_stamp_ns = HeaderStampToNanoseconds(cloud.header.stamp),
      .width = cloud.width,
      .height = cloud.height,
      .point_step = cloud.point_step,
      .data = cloud.data.empty() ? nullptr : cloud.data.data(),
      .data_size = cloud.data.size(),
  };
}

}  // namespace causal_slam::ros_adapters
