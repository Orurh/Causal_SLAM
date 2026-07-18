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
    causal_slam::pointcloud::PointCloud2FieldInfo field_info;
    field_info.name = field.name;
    field_info.offset = field.offset;
    field_info.datatype = field.datatype;
    field_info.count = field.count;
    fields.push_back(field_info);
  }

  return fields;
}

causal_slam::pointcloud::PointCloud2CloudView ToPointCloud2CloudView(const sensor_msgs::msg::PointCloud2& cloud) {
  causal_slam::pointcloud::PointCloud2CloudView view;
  view.header_stamp_ns = HeaderStampToNanoseconds(cloud.header.stamp);
  view.width = cloud.width;
  view.height = cloud.height;
  view.point_step = cloud.point_step;
  view.data = cloud.data.empty() ? nullptr : cloud.data.data();
  view.data_size = cloud.data.size();
  return view;
}

}  // namespace causal_slam::ros_adapters
