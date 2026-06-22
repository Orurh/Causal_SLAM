#include "pointcloud/point_cloud2_time_field_extractor.h"

#include "pointcloud/point_cloud2_datatype.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <vector>

namespace causal_slam::pointcloud {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr double kNanosecondsPerSecondDouble = 1'000'000'000.0;

std::optional<std::int64_t> SecondsToNanoseconds(double seconds) {
  if (!std::isfinite(seconds)) {
    return std::nullopt;
  }

  constexpr double max_seconds =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) /
      kNanosecondsPerSecondDouble;
  constexpr double min_seconds =
      static_cast<double>(std::numeric_limits<std::int64_t>::min()) /
      kNanosecondsPerSecondDouble;

  if (seconds > max_seconds || seconds < min_seconds) {
    return std::nullopt;
  }

  return static_cast<std::int64_t>(
      std::llround(seconds * kNanosecondsPerSecondDouble));
}

std::uint32_t PointCount(const PointCloud2CloudView& cloud) {
  const std::uint64_t point_count =
      static_cast<std::uint64_t>(cloud.width) *
      static_cast<std::uint64_t>(cloud.height);

  if (point_count > std::numeric_limits<std::uint32_t>::max()) {
    return std::numeric_limits<std::uint32_t>::max();
  }

  return static_cast<std::uint32_t>(point_count);
}

bool HasEnoughData(const PointCloud2CloudView& cloud, std::uint32_t point_count) {
  if (point_count == 0) {
    return true;
  }

  const std::size_t required_size =
      static_cast<std::size_t>(point_count) *
      static_cast<std::size_t>(cloud.point_step);

  return cloud.data != nullptr && cloud.data_size >= required_size;
}

template <typename T>
T ReadScalarUnchecked(
    const PointCloud2CloudView& cloud,
    std::uint32_t point_index,
    std::uint32_t field_offset) {
  const std::size_t byte_offset =
      static_cast<std::size_t>(point_index) *
          static_cast<std::size_t>(cloud.point_step) +
      static_cast<std::size_t>(field_offset);

  T value{};
  std::memcpy(&value, cloud.data + byte_offset, sizeof(T));
  return value;
}

bool FieldFitsPointStep(
    const PointCloud2FieldInfo& time_field,
    std::uint32_t point_step,
    std::size_t value_size) {
  return static_cast<std::size_t>(time_field.offset) + value_size <=
         static_cast<std::size_t>(point_step);
}

std::optional<std::int64_t> ReadAbsolutePointTimeNs(
    const PointCloud2CloudView& cloud,
    const PointCloud2FieldInfo& time_field,
    std::uint32_t point_index) {
  switch (time_field.datatype) {
    case kPointCloud2Float64:
      return SecondsToNanoseconds(
          ReadScalarUnchecked<double>(cloud, point_index, time_field.offset));

    case kPointCloud2Float32:
      return SecondsToNanoseconds(static_cast<double>(
          ReadScalarUnchecked<float>(cloud, point_index, time_field.offset)));

    default:
      return std::nullopt;
  }
}

std::optional<std::int64_t> ReadRelativeOffsetNs(
    const PointCloud2CloudView& cloud,
    const PointCloud2FieldInfo& time_field,
    std::uint32_t point_index) {
  switch (time_field.datatype) {
    case kPointCloud2Uint32:
      return static_cast<std::int64_t>(
          ReadScalarUnchecked<std::uint32_t>(
              cloud, point_index, time_field.offset));

    case kPointCloud2Int32:
      return static_cast<std::int64_t>(
          ReadScalarUnchecked<std::int32_t>(
              cloud, point_index, time_field.offset));

    case kPointCloud2Float64:
      return SecondsToNanoseconds(
          ReadScalarUnchecked<double>(cloud, point_index, time_field.offset));

    case kPointCloud2Float32:
      return SecondsToNanoseconds(static_cast<double>(
          ReadScalarUnchecked<float>(cloud, point_index, time_field.offset)));

    default:
      return std::nullopt;
  }
}

std::optional<std::size_t> TimeFieldValueSize(
    const PointCloud2FieldInfo& time_field) {
  switch (time_field.datatype) {
    case kPointCloud2Float64:
      return sizeof(double);
    case kPointCloud2Float32:
      return sizeof(float);
    case kPointCloud2Uint32:
      return sizeof(std::uint32_t);
    case kPointCloud2Int32:
      return sizeof(std::int32_t);
    default:
      return std::nullopt;
  }
}

PointCloud2TimeFieldUnit UnitForField(const PointCloud2FieldInfo& time_field) {
  if (time_field.time_role == PointCloud2TimeFieldRole::kPointTime) {
    return PointCloud2TimeFieldUnit::kAbsoluteSeconds;
  }

  if (time_field.time_role == PointCloud2TimeFieldRole::kPointOffsetTime) {
    switch (time_field.datatype) {
      case kPointCloud2Float32:
      case kPointCloud2Float64:
        return PointCloud2TimeFieldUnit::kRelativeSeconds;

      case kPointCloud2Int32:
      case kPointCloud2Uint32:
        return PointCloud2TimeFieldUnit::kRelativeNanoseconds;

      default:
        return PointCloud2TimeFieldUnit::kUnknown;
    }
  }

  return PointCloud2TimeFieldUnit::kUnknown;
}

}  // namespace

const char* ToString(PointCloud2TimeFieldUnit unit) {
  switch (unit) {
    case PointCloud2TimeFieldUnit::kUnknown:
      return "unknown";
    case PointCloud2TimeFieldUnit::kAbsoluteSeconds:
      return "absolute_seconds";
    case PointCloud2TimeFieldUnit::kRelativeNanoseconds:
      return "relative_nanoseconds";
    case PointCloud2TimeFieldUnit::kRelativeSeconds:
      return "relative_seconds";
  }

  return "unknown";
}

PointCloud2TimeFieldExtraction PointCloud2TimeFieldExtractor::Extract(
    const PointCloud2CloudView& cloud,
    const PointCloud2FieldInfo& time_field) const {
  PointCloud2TimeFieldExtraction extraction;
  extraction.time_role = time_field.time_role;
  extraction.time_unit = UnitForField(time_field);
  extraction.point_count_total = PointCount(cloud);

  if (time_field.time_role != PointCloud2TimeFieldRole::kPointTime &&
      time_field.time_role != PointCloud2TimeFieldRole::kPointOffsetTime) {
    extraction.reason = "unsupported_time_field_role";
    return extraction;
  }

  if (time_field.time_role == PointCloud2TimeFieldRole::kPointTime &&
      time_field.datatype == kPointCloud2Float32) {
    extraction.reason = "absolute_float32_timestamp_precision_unsafe";
    return extraction;
  }

  if (extraction.point_count_total == 0) {
    extraction.reason = "empty_cloud";
    return extraction;
  }

  if (cloud.point_step == 0) {
    extraction.reason = "invalid_point_step";
    return extraction;
  }

  const std::optional<std::size_t> value_size = TimeFieldValueSize(time_field);
  if (!value_size.has_value()) {
    extraction.reason = "unsupported_time_field_datatype";
    return extraction;
  }

  if (!FieldFitsPointStep(time_field, cloud.point_step, *value_size)) {
    extraction.reason = "time_field_exceeds_point_step";
    return extraction;
  }

  if (!HasEnoughData(cloud, extraction.point_count_total)) {
    extraction.reason = "cloud_data_too_small";
    return extraction;
  }

  std::vector<std::int64_t> point_times_ns;
  point_times_ns.reserve(extraction.point_count_total);

  for (std::uint32_t point_index = 0; point_index < extraction.point_count_total;
       ++point_index) {
    std::optional<std::int64_t> point_time_ns;

    if (time_field.time_role == PointCloud2TimeFieldRole::kPointTime) {
      point_time_ns = ReadAbsolutePointTimeNs(cloud, time_field, point_index);
    } else if (time_field.time_role == PointCloud2TimeFieldRole::kPointOffsetTime) {
      const std::optional<std::int64_t> offset_ns =
          ReadRelativeOffsetNs(cloud, time_field, point_index);

      if (offset_ns.has_value()) {
        point_time_ns = cloud.header_stamp_ns + *offset_ns;
      }
    }

    if (point_time_ns.has_value()) {
      point_times_ns.push_back(*point_time_ns);
    }
  }

  extraction.point_count_used = static_cast<std::uint32_t>(point_times_ns.size());

  if (point_times_ns.empty()) {
    extraction.reason = "no_valid_point_timestamps";
    return extraction;
  }

  const auto [min_it, max_it] =
      std::minmax_element(point_times_ns.begin(), point_times_ns.end());

  extraction.scan_window = causal_slam::core::TimeWindow{
      .start_ns = *min_it,
      .end_ns = *max_it,
  };

  extraction.has_scan_window = extraction.scan_window.IsValid();
  extraction.reason = extraction.has_scan_window
                          ? "point_time_field_extracted"
                          : "invalid_extracted_scan_window";

  return extraction;
}

}  // namespace causal_slam::pointcloud
