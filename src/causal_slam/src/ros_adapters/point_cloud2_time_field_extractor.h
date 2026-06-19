#pragma once

#include <cstdint>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "core/time_window.h"
#include "ros_adapters/point_cloud2_field_inspector.h"

namespace causal_slam::ros_adapters {

enum class PointCloud2TimeFieldUnit : std::uint8_t {
  kUnknown,
  kAbsoluteSeconds,
  kRelativeNanoseconds,
  kRelativeSeconds,
};

[[nodiscard]] const char* ToString(PointCloud2TimeFieldUnit unit);

struct PointCloud2TimeFieldExtraction {
  bool has_scan_window{false};

  causal_slam::core::TimeWindow scan_window;

  std::uint32_t point_count_total{0};
  std::uint32_t point_count_used{0};

  PointCloud2TimeFieldRole time_role{PointCloud2TimeFieldRole::kNone};
  PointCloud2TimeFieldUnit time_unit{PointCloud2TimeFieldUnit::kUnknown};

  std::string reason{"not_extracted"};
};

class PointCloud2TimeFieldExtractor final {
 public:
  [[nodiscard]] PointCloud2TimeFieldExtraction Extract(
      const sensor_msgs::msg::PointCloud2& cloud,
      const PointCloud2FieldInfo& time_field) const;
};

}  // namespace causal_slam::ros_adapters