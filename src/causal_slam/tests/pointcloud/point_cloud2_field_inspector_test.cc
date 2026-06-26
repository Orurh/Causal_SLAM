#include "domain/sensors/pointcloud/point_cloud2_field_inspector.h"

#include "domain/sensors/pointcloud/point_cloud2_datatype.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace causal_slam::pointcloud {
namespace {

PointCloud2FieldInfo Field(
    std::string name,
    std::uint32_t offset,
    std::uint8_t datatype,
    std::uint32_t count = 1) {
  return PointCloud2FieldInfo{
      .name = std::move(name),
      .offset = offset,
      .datatype = datatype,
      .count = count,
  };
}

TEST(PointCloud2FieldInspectorCoreTest, EmptyFieldsHaveNoTimeCandidate) {
  const PointCloud2FieldInspector inspector;

  const auto inspection = inspector.Inspect({});

  EXPECT_TRUE(inspection.fields.empty());
  EXPECT_FALSE(inspection.has_time_candidate);
  EXPECT_FALSE(inspection.has_supported_time_field);
  EXPECT_FALSE(inspection.primary_time_field.has_value());
  EXPECT_EQ(inspection.reason, "no_time_field_candidate");
}

TEST(PointCloud2FieldInspectorCoreTest, DetectsFloat64TimestampAsSupported) {
  const std::vector<PointCloud2FieldInfo> fields{
      Field("x", 0, kPointCloud2Float32),
      Field("y", 4, kPointCloud2Float32),
      Field("z", 8, kPointCloud2Float32),
      Field("timestamp", 16, kPointCloud2Float64),
  };

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(fields);

  ASSERT_TRUE(inspection.primary_time_field.has_value());
  EXPECT_TRUE(inspection.has_time_candidate);
  EXPECT_TRUE(inspection.has_supported_time_field);
  EXPECT_EQ(inspection.primary_time_field->name, "timestamp");
  EXPECT_EQ(inspection.primary_time_field->time_role,
            PointCloud2TimeFieldRole::kPointTime);
  EXPECT_EQ(inspection.reason, "supported_point_time_field_detected");
}

TEST(PointCloud2FieldInspectorCoreTest, DetectsUint32OffsetTimeAsSupported) {
  const std::vector<PointCloud2FieldInfo> fields{
      Field("x", 0, kPointCloud2Float32),
      Field("offset_time", 12, kPointCloud2Uint32),
  };

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(fields);

  ASSERT_TRUE(inspection.primary_time_field.has_value());
  EXPECT_TRUE(inspection.has_time_candidate);
  EXPECT_TRUE(inspection.has_supported_time_field);
  EXPECT_EQ(inspection.primary_time_field->name, "offset_time");
  EXPECT_EQ(inspection.primary_time_field->time_role,
            PointCloud2TimeFieldRole::kPointOffsetTime);
  EXPECT_EQ(inspection.reason, "supported_point_time_field_detected");
}

TEST(PointCloud2FieldInspectorCoreTest, RejectsFloat32AbsoluteTimestamp) {
  const std::vector<PointCloud2FieldInfo> fields{
      Field("x", 0, kPointCloud2Float32),
      Field("timestamp", 12, kPointCloud2Float32),
  };

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(fields);

  EXPECT_TRUE(inspection.has_time_candidate);
  EXPECT_FALSE(inspection.has_supported_time_field);
  EXPECT_FALSE(inspection.primary_time_field.has_value());
  EXPECT_EQ(inspection.reason, "absolute_float32_timestamp_precision_unsafe");
}

TEST(PointCloud2FieldInspectorCoreTest, DetectsSplitTimePairButDoesNotSupportItYet) {
  const std::vector<PointCloud2FieldInfo> fields{
      Field("timeSecond", 16, kPointCloud2Uint32),
      Field("timeNanosecond", 20, kPointCloud2Uint32),
  };

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(fields);

  EXPECT_TRUE(inspection.has_time_candidate);
  EXPECT_FALSE(inspection.has_supported_time_field);
  EXPECT_FALSE(inspection.primary_time_field.has_value());
  EXPECT_EQ(inspection.reason, "split_time_fields_detected_not_supported_yet");
}

TEST(PointCloud2FieldInspectorCoreTest, IgnoresNonTimeFields) {
  const std::vector<PointCloud2FieldInfo> fields{
      Field("x", 0, kPointCloud2Float32),
      Field("y", 4, kPointCloud2Float32),
      Field("z", 8, kPointCloud2Float32),
      Field("intensity", 12, kPointCloud2Float32),
      Field("ring", 16, kPointCloud2Uint16),
  };

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(fields);

  EXPECT_FALSE(inspection.has_time_candidate);
  EXPECT_FALSE(inspection.has_supported_time_field);
  EXPECT_FALSE(inspection.primary_time_field.has_value());
  EXPECT_EQ(inspection.reason, "no_time_field_candidate");
}

}  // namespace
}  // namespace causal_slam::pointcloud
