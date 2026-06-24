#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "domain/time/time_window.h"
#include "pointcloud/point_cloud2_field_inspector.h"

namespace causal_slam::pointcloud {

enum class PointCloud2TimeFieldUnit : std::uint8_t {
  kUnknown,
  kAbsoluteSeconds,
  kRelativeNanoseconds,
  kRelativeSeconds,
};

[[nodiscard]] const char* ToString(PointCloud2TimeFieldUnit unit);

struct PointCloud2CloudView {
  std::int64_t header_stamp_ns{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t point_step{0};

  const std::uint8_t* data{nullptr};
  std::size_t data_size{0};
};

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
      const PointCloud2CloudView& cloud,
      const PointCloud2FieldInfo& time_field) const;
};

}  // namespace causal_slam::pointcloud
