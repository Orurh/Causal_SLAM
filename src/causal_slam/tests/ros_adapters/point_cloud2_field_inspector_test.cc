#include "ros_adapters/point_cloud2_field_inspector.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

namespace causal_slam::ros_adapters {
namespace {

sensor_msgs::msg::PointField Field(
    std::string name,
    std::uint32_t offset,
    std::uint8_t datatype,
    std::uint32_t count = 1) {
  sensor_msgs::msg::PointField field;
  field.name = std::move(name);
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

TEST(PointCloud2FieldInspectorTest, EmptyCloudHasNoTimeCandidate) {
  const sensor_msgs::msg::PointCloud2 cloud;

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(cloud);

  EXPECT_TRUE(inspection.fields.empty());
  EXPECT_FALSE(inspection.has_time_candidate);
  EXPECT_FALSE(inspection.has_supported_time_field);
  EXPECT_FALSE(inspection.primary_time_field.has_value());
  EXPECT_EQ(inspection.reason, "no_time_field_candidate");
}

TEST(PointCloud2FieldInspectorTest, DetectsTimestampFloat64AsSupportedTimeField) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields.push_back(Field("x", 0, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(Field("y", 4, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(Field("z", 8, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(
      Field("timestamp", 16, sensor_msgs::msg::PointField::FLOAT64));

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(cloud);

  ASSERT_TRUE(inspection.primary_time_field.has_value());
  EXPECT_TRUE(inspection.has_time_candidate);
  EXPECT_TRUE(inspection.has_supported_time_field);
  EXPECT_EQ(inspection.primary_time_field->name, "timestamp");
  EXPECT_EQ(inspection.primary_time_field->time_role,
            PointCloud2TimeFieldRole::kPointTime);
  EXPECT_EQ(inspection.reason, "supported_point_time_field_detected");
}

TEST(PointCloud2FieldInspectorTest, DetectsOffsetTimeUint32AsSupportedTimeField) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields.push_back(Field("x", 0, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(
      Field("offset_time", 12, sensor_msgs::msg::PointField::UINT32));

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(cloud);

  ASSERT_TRUE(inspection.primary_time_field.has_value());
  EXPECT_TRUE(inspection.has_time_candidate);
  EXPECT_TRUE(inspection.has_supported_time_field);
  EXPECT_EQ(inspection.primary_time_field->name, "offset_time");
  EXPECT_EQ(inspection.primary_time_field->time_role,
            PointCloud2TimeFieldRole::kPointOffsetTime);
}

TEST(PointCloud2FieldInspectorTest, DetectsSplitTimePairButDoesNotSupportItYet) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields.push_back(
      Field("timeSecond", 16, sensor_msgs::msg::PointField::UINT32));
  cloud.fields.push_back(
      Field("timeNanosecond", 20, sensor_msgs::msg::PointField::UINT32));

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(cloud);

  EXPECT_TRUE(inspection.has_time_candidate);
  EXPECT_FALSE(inspection.has_supported_time_field);
  EXPECT_FALSE(inspection.primary_time_field.has_value());
  EXPECT_EQ(inspection.reason, "split_time_fields_detected_not_supported_yet");
}

TEST(PointCloud2FieldInspectorTest, IgnoresNonTimeFields) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields.push_back(Field("x", 0, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(Field("y", 4, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(Field("z", 8, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(Field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32));
  cloud.fields.push_back(Field("ring", 16, sensor_msgs::msg::PointField::UINT16));

  const PointCloud2FieldInspector inspector;
  const auto inspection = inspector.Inspect(cloud);

  EXPECT_FALSE(inspection.has_time_candidate);
  EXPECT_FALSE(inspection.has_supported_time_field);
  EXPECT_FALSE(inspection.primary_time_field.has_value());
  EXPECT_EQ(inspection.reason, "no_time_field_candidate");
}

}  // namespace
}  // namespace causal_slam::ros_adapters