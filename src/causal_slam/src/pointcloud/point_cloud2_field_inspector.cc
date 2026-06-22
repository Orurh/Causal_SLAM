#include "pointcloud/point_cloud2_field_inspector.h"

#include "pointcloud/point_cloud2_datatype.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>
#include <utility>

namespace causal_slam::pointcloud {
namespace {

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });

  return value;
}

bool IsAbsoluteFloat32Timestamp(const PointCloud2FieldInfo& field_info) {
  return field_info.time_role == PointCloud2TimeFieldRole::kPointTime &&
         field_info.datatype == kPointCloud2Float32;
}

bool IsSupportedTimeField(const PointCloud2FieldInfo& field_info) {
  if (field_info.time_role == PointCloud2TimeFieldRole::kPointTime) {
    return field_info.datatype == kPointCloud2Float64;
  }

  if (field_info.time_role == PointCloud2TimeFieldRole::kPointOffsetTime) {
    switch (field_info.datatype) {
      case kPointCloud2Float32:
      case kPointCloud2Float64:
      case kPointCloud2Int32:
      case kPointCloud2Uint32:
        return true;
      default:
        return false;
    }
  }

  return false;
}

bool IsSimpleTimeRole(PointCloud2TimeFieldRole role) {
  return role == PointCloud2TimeFieldRole::kPointTime ||
         role == PointCloud2TimeFieldRole::kPointOffsetTime;
}

PointCloud2TimeFieldRole DetectTimeFieldRole(const std::string& field_name) {
  const std::string name = ToLower(field_name);

  if (name == "timestamp" || name == "time" || name == "t") {
    return PointCloud2TimeFieldRole::kPointTime;
  }

  if (name == "offset_time" || name == "time_offset" || name == "offsettime") {
    return PointCloud2TimeFieldRole::kPointOffsetTime;
  }

  if (name == "timesecond" || name == "time_second" || name == "time_s") {
    return PointCloud2TimeFieldRole::kSplitTimeSecond;
  }

  if (name == "timenanosecond" || name == "time_nanosecond" ||
      name == "time_ns") {
    return PointCloud2TimeFieldRole::kSplitTimeNanosecond;
  }

  return PointCloud2TimeFieldRole::kNone;
}

}  // namespace

const char* ToString(PointCloud2TimeFieldRole role) {
  switch (role) {
    case PointCloud2TimeFieldRole::kNone:
      return "none";
    case PointCloud2TimeFieldRole::kPointTime:
      return "point_time";
    case PointCloud2TimeFieldRole::kPointOffsetTime:
      return "point_offset_time";
    case PointCloud2TimeFieldRole::kSplitTimeSecond:
      return "split_time_second";
    case PointCloud2TimeFieldRole::kSplitTimeNanosecond:
      return "split_time_nanosecond";
  }

  return "unknown";
}

const char* PointCloud2DatatypeToString(std::uint8_t datatype) {
  switch (datatype) {
    case kPointCloud2Int8:
      return "INT8";
    case kPointCloud2Uint8:
      return "UINT8";
    case kPointCloud2Int16:
      return "INT16";
    case kPointCloud2Uint16:
      return "UINT16";
    case kPointCloud2Int32:
      return "INT32";
    case kPointCloud2Uint32:
      return "UINT32";
    case kPointCloud2Float32:
      return "FLOAT32";
    case kPointCloud2Float64:
      return "FLOAT64";
    default:
      return "UNKNOWN";
  }
}

PointCloud2FieldInspection PointCloud2FieldInspector::Inspect(
    const std::vector<PointCloud2FieldInfo>& fields) const {
  PointCloud2FieldInspection inspection;
  inspection.fields.reserve(fields.size());

  bool has_split_time_second = false;
  bool has_split_time_nanosecond = false;
  bool has_unsafe_absolute_float32_timestamp = false;
  bool has_unsupported_simple_time_field = false;

  for (auto field_info : fields) {
    field_info.time_role = DetectTimeFieldRole(field_info.name);

    if (field_info.time_role != PointCloud2TimeFieldRole::kNone) {
      inspection.has_time_candidate = true;
    }

    if (field_info.time_role == PointCloud2TimeFieldRole::kSplitTimeSecond) {
      has_split_time_second = true;
    }

    if (field_info.time_role == PointCloud2TimeFieldRole::kSplitTimeNanosecond) {
      has_split_time_nanosecond = true;
    }

    if (!inspection.primary_time_field.has_value() &&
        IsSimpleTimeRole(field_info.time_role)) {
      if (IsAbsoluteFloat32Timestamp(field_info)) {
        has_unsafe_absolute_float32_timestamp = true;
      } else if (IsSupportedTimeField(field_info)) {
        inspection.has_supported_time_field = true;
        inspection.primary_time_field = field_info;
      } else {
        has_unsupported_simple_time_field = true;
      }
    }

    inspection.fields.push_back(std::move(field_info));
  }

  if (inspection.primary_time_field.has_value()) {
    inspection.reason = "supported_point_time_field_detected";
    return inspection;
  }

  if (has_unsafe_absolute_float32_timestamp) {
    inspection.reason = "absolute_float32_timestamp_precision_unsafe";
    return inspection;
  }

  if (has_split_time_second && has_split_time_nanosecond) {
    inspection.reason = "split_time_fields_detected_not_supported_yet";
    return inspection;
  }

  if (has_unsupported_simple_time_field) {
    inspection.reason = "unsupported_time_field_datatype";
    return inspection;
  }

  if (inspection.has_time_candidate) {
    inspection.reason = "time_field_candidate_detected_not_supported_yet";
    return inspection;
  }

  inspection.reason = "no_time_field_candidate";
  return inspection;
}

}  // namespace causal_slam::pointcloud
