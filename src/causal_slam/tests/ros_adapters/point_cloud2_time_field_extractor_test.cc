#include "ros_adapters/point_cloud2_time_field_extractor.h"

#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace causal_slam::ros_adapters {
namespace {

sensor_msgs::msg::PointField Field(std::string name,
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

template <typename T>
void WriteValue(sensor_msgs::msg::PointCloud2& cloud,
                std::uint32_t point_index,
                std::uint32_t field_offset,
                const T& value) {
  const std::size_t byte_offset =
      static_cast<std::size_t>(point_index) *
          static_cast<std::size_t>(cloud.point_step) +
      static_cast<std::size_t>(field_offset);

  std::memcpy(cloud.data.data() + byte_offset, &value, sizeof(T));
}

sensor_msgs::msg::PointCloud2 MakeCloud(std::uint32_t point_count,
                                         std::uint32_t point_step) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = point_count;
  cloud.point_step = point_step;
  cloud.row_step = point_step * point_count;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step);
  return cloud;
}

PointCloud2FieldInfo TimestampFloat64Field(std::uint32_t offset) {
  return PointCloud2FieldInfo{
      .name = "timestamp",
      .offset = offset,
      .datatype = sensor_msgs::msg::PointField::FLOAT64,
      .count = 1,
      .time_role = PointCloud2TimeFieldRole::kPointTime,
  };
}

PointCloud2FieldInfo OffsetTimeUint32Field(std::uint32_t offset) {
  return PointCloud2FieldInfo{
      .name = "offset_time",
      .offset = offset,
      .datatype = sensor_msgs::msg::PointField::UINT32,
      .count = 1,
      .time_role = PointCloud2TimeFieldRole::kPointOffsetTime,
  };
}

TEST(PointCloud2TimeFieldExtractorTest, ExtractsAbsoluteTimestampFloat64Window) {
  sensor_msgs::msg::PointCloud2 cloud = MakeCloud(3, sizeof(double));
  cloud.fields.push_back(
      Field("timestamp", 0, sensor_msgs::msg::PointField::FLOAT64));

  WriteValue(cloud, 0, 0, 10.000);
  WriteValue(cloud, 1, 0, 10.050);
  WriteValue(cloud, 2, 0, 10.100);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(cloud, TimestampFloat64Field(0));

  EXPECT_TRUE(result.has_scan_window);
  EXPECT_EQ(result.scan_window.start_ns, 10'000'000'000LL);
  EXPECT_EQ(result.scan_window.end_ns, 10'100'000'000LL);
  EXPECT_EQ(result.point_count_total, 3U);
  EXPECT_EQ(result.point_count_used, 3U);
  EXPECT_EQ(result.time_unit, PointCloud2TimeFieldUnit::kAbsoluteSeconds);
  EXPECT_EQ(result.reason, "point_time_field_extracted");
}

TEST(PointCloud2TimeFieldExtractorTest, ExtractsOffsetTimeUint32WindowFromHeaderStamp) {
  sensor_msgs::msg::PointCloud2 cloud = MakeCloud(3, sizeof(std::uint32_t));
  cloud.header.stamp.sec = 20;
  cloud.header.stamp.nanosec = 100'000'000;
  cloud.fields.push_back(
      Field("offset_time", 0, sensor_msgs::msg::PointField::UINT32));

  WriteValue<std::uint32_t>(cloud, 0, 0, 0U);
  WriteValue<std::uint32_t>(cloud, 1, 0, 50'000'000U);
  WriteValue<std::uint32_t>(cloud, 2, 0, 100'000'000U);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(cloud, OffsetTimeUint32Field(0));

  EXPECT_TRUE(result.has_scan_window);
  EXPECT_EQ(result.scan_window.start_ns, 20'100'000'000LL);
  EXPECT_EQ(result.scan_window.end_ns, 20'200'000'000LL);
  EXPECT_EQ(result.point_count_total, 3U);
  EXPECT_EQ(result.point_count_used, 3U);
  EXPECT_EQ(result.time_unit, PointCloud2TimeFieldUnit::kRelativeNanoseconds);
  EXPECT_EQ(result.reason, "point_time_field_extracted");
}

TEST(PointCloud2TimeFieldExtractorTest, EmptyCloudDoesNotExtractWindow) {
  sensor_msgs::msg::PointCloud2 cloud = MakeCloud(0, sizeof(double));

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(cloud, TimestampFloat64Field(0));

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.point_count_total, 0U);
  EXPECT_EQ(result.point_count_used, 0U);
  EXPECT_EQ(result.reason, "empty_cloud");
}

TEST(PointCloud2TimeFieldExtractorTest, UnsupportedSplitTimeRoleDoesNotExtractWindow) {
  sensor_msgs::msg::PointCloud2 cloud = MakeCloud(3, sizeof(std::uint32_t));

  const auto split_field = PointCloud2FieldInfo{
      .name = "timeSecond",
      .offset = 0,
      .datatype = sensor_msgs::msg::PointField::UINT32,
      .count = 1,
      .time_role = PointCloud2TimeFieldRole::kSplitTimeSecond,
  };

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(cloud, split_field);

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.reason, "unsupported_time_field_role");
}

TEST(PointCloud2TimeFieldExtractorTest, DataTooSmallDoesNotExtractWindow) {
  sensor_msgs::msg::PointCloud2 cloud = MakeCloud(3, sizeof(double));
  cloud.data.resize(sizeof(double));

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(cloud, TimestampFloat64Field(0));

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.reason, "cloud_data_too_small");
}

TEST(PointCloud2TimeFieldExtractorTest, FieldOutsidePointStepDoesNotExtractWindow) {
  sensor_msgs::msg::PointCloud2 cloud = MakeCloud(3, sizeof(float));

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(cloud, TimestampFloat64Field(0));

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.reason, "time_field_exceeds_point_step");
}

}  // namespace
}  // namespace causal_slam::ros_adapters