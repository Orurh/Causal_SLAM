#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace causal_slam::pointcloud {

enum class PointCloud2TimeFieldRole : std::uint8_t {
  kNone,
  kPointTime,
  kPointOffsetTime,
  kSplitTimeSecond,
  kSplitTimeNanosecond,
};

enum class PointCloud2TimeFieldOverrideMode : std::uint8_t {
  kAuto,
  kExplicit,
  kDisabled,
};

enum class PointCloud2TimeFieldOverrideInterpretation : std::uint8_t {
  kAuto,
  kAbsolute,
  kRelative,
};

enum class PointCloud2TimeFieldOverrideUnit : std::uint8_t {
  kAuto,
  kSeconds,
  kNanoseconds,
};

struct PointCloud2TimeFieldOverrideConfig {
  PointCloud2TimeFieldOverrideMode mode{PointCloud2TimeFieldOverrideMode::kAuto};
  std::string field_name;
  PointCloud2TimeFieldOverrideInterpretation interpretation{PointCloud2TimeFieldOverrideInterpretation::kAuto};
  PointCloud2TimeFieldOverrideUnit unit{PointCloud2TimeFieldOverrideUnit::kAuto};
};

[[nodiscard]] const char* ToString(PointCloud2TimeFieldRole role);
[[nodiscard]] const char* PointCloud2DatatypeToString(std::uint8_t datatype);

struct PointCloud2FieldInfo {
  std::string name;
  std::uint32_t offset{0};
  std::uint8_t datatype{0};
  std::uint32_t count{0};

  PointCloud2TimeFieldRole time_role{PointCloud2TimeFieldRole::kNone};
};

struct PointCloud2FieldInspection {
  std::vector<PointCloud2FieldInfo> fields;

  bool has_time_candidate{false};
  bool has_supported_time_field{false};

  std::optional<PointCloud2FieldInfo> primary_time_field;

  std::string reason{"no_time_field_candidate"};
};

class PointCloud2FieldInspector final {
 public:
  [[nodiscard]] PointCloud2FieldInspection Inspect(
      const std::vector<PointCloud2FieldInfo>& fields,
      const PointCloud2TimeFieldOverrideConfig& config = PointCloud2TimeFieldOverrideConfig{}) const;
};

}  // namespace causal_slam::pointcloud
