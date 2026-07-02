#include "domain/sensors/pointcloud/point_cloud2_time_field_extractor.h"

#include "domain/sensors/pointcloud/point_cloud2_datatype.h"

#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

namespace causal_slam::pointcloud {
namespace {

template <typename T>
void WriteValue(std::vector<std::uint8_t>* data, std::uint32_t point_step, std::uint32_t point_index, std::uint32_t field_offset,
                const T& value) {
  const std::size_t byte_offset =
      static_cast<std::size_t>(point_index) * static_cast<std::size_t>(point_step) + static_cast<std::size_t>(field_offset);

  std::memcpy(data->data() + byte_offset, &value, sizeof(T));
}

PointCloud2CloudView MakeView(std::int64_t header_stamp_ns, std::uint32_t point_count, std::uint32_t point_step,
                              const std::vector<std::uint8_t>& data) {
  return PointCloud2CloudView{
      .header_stamp_ns = header_stamp_ns,
      .width = point_count,
      .height = 1,
      .point_step = point_step,
      .data = data.empty() ? nullptr : data.data(),
      .data_size = data.size(),
  };
}

PointCloud2FieldInfo TimestampFloat64Field(std::uint32_t offset) {
  return PointCloud2FieldInfo{
      .name = "timestamp",
      .offset = offset,
      .datatype = kPointCloud2Float64,
      .count = 1,
      .time_role = PointCloud2TimeFieldRole::kPointTime,
  };
}

PointCloud2FieldInfo TimestampFloat32Field(std::uint32_t offset) {
  return PointCloud2FieldInfo{
      .name = "timestamp",
      .offset = offset,
      .datatype = kPointCloud2Float32,
      .count = 1,
      .time_role = PointCloud2TimeFieldRole::kPointTime,
  };
}

PointCloud2FieldInfo OffsetTimeUint32Field(std::uint32_t offset) {
  return PointCloud2FieldInfo{
      .name = "offset_time",
      .offset = offset,
      .datatype = kPointCloud2Uint32,
      .count = 1,
      .time_role = PointCloud2TimeFieldRole::kPointOffsetTime,
  };
}

PointCloud2FieldInfo OusterTUint32Field(std::uint32_t offset) {
  return PointCloud2FieldInfo{
      .name = "t",
      .offset = offset,
      .datatype = kPointCloud2Uint32,
      .count = 1,
      .time_role = PointCloud2TimeFieldRole::kPointOffsetTime,
  };
}

TEST(PointCloud2TimeFieldExtractorCoreTest, ExtractsAbsoluteTimestampFloat64Window) {
  constexpr std::uint32_t point_count = 3;
  constexpr std::uint32_t point_step = sizeof(double);

  std::vector<std::uint8_t> data(point_count * point_step);

  WriteValue(&data, point_step, 0, 0, 10.000);
  WriteValue(&data, point_step, 1, 0, 10.050);
  WriteValue(&data, point_step, 2, 0, 10.100);

  const auto view = MakeView(0, point_count, point_step, data);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(view, TimestampFloat64Field(0));

  EXPECT_TRUE(result.has_scan_window);
  EXPECT_EQ(result.scan_window.start_ns, 10'000'000'000LL);
  EXPECT_EQ(result.scan_window.end_ns, 10'100'000'000LL);
  EXPECT_EQ(result.point_count_total, 3U);
  EXPECT_EQ(result.point_count_used, 3U);
  EXPECT_EQ(result.time_unit, PointCloud2TimeFieldUnit::kAbsoluteSeconds);
  EXPECT_EQ(result.reason, "point_time_field_extracted");
}

TEST(PointCloud2TimeFieldExtractorCoreTest, ExtractsOffsetTimeUint32WindowFromHeaderStamp) {
  constexpr std::int64_t header_stamp_ns = 20'100'000'000LL;
  constexpr std::uint32_t point_count = 3;
  constexpr std::uint32_t point_step = sizeof(std::uint32_t);

  std::vector<std::uint8_t> data(point_count * point_step);

  WriteValue<std::uint32_t>(&data, point_step, 0, 0, 0U);
  WriteValue<std::uint32_t>(&data, point_step, 1, 0, 50'000'000U);
  WriteValue<std::uint32_t>(&data, point_step, 2, 0, 100'000'000U);

  const auto view = MakeView(header_stamp_ns, point_count, point_step, data);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(view, OffsetTimeUint32Field(0));

  EXPECT_TRUE(result.has_scan_window);
  EXPECT_EQ(result.scan_window.start_ns, 20'100'000'000LL);
  EXPECT_EQ(result.scan_window.end_ns, 20'200'000'000LL);
  EXPECT_EQ(result.point_count_total, 3U);
  EXPECT_EQ(result.point_count_used, 3U);
  EXPECT_EQ(result.time_unit, PointCloud2TimeFieldUnit::kRelativeNanoseconds);
  EXPECT_EQ(result.reason, "point_time_field_extracted");
}

TEST(PointCloud2TimeFieldExtractorCoreTest, ExtractsOusterUint32TWindowFromHeaderStamp) {
  constexpr std::int64_t header_stamp_ns = 360'000'000'000LL;
  constexpr std::uint32_t point_count = 3;
  constexpr std::uint32_t point_step = 36;
  constexpr std::uint32_t t_offset = 20;

  std::vector<std::uint8_t> data(point_count * point_step);

  WriteValue<std::uint32_t>(&data, point_step, 0, t_offset, 1'000U);
  WriteValue<std::uint32_t>(&data, point_step, 1, t_offset, 50'000'000U);
  WriteValue<std::uint32_t>(&data, point_step, 2, t_offset, 99'000'000U);

  const auto view = MakeView(header_stamp_ns, point_count, point_step, data);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(view, OusterTUint32Field(t_offset));

  EXPECT_TRUE(result.has_scan_window);
  EXPECT_EQ(result.scan_window.start_ns, 360'000'001'000LL);
  EXPECT_EQ(result.scan_window.end_ns, 360'099'000'000LL);
  EXPECT_EQ(result.point_count_total, 3U);
  EXPECT_EQ(result.point_count_used, 3U);
  EXPECT_EQ(result.time_unit, PointCloud2TimeFieldUnit::kRelativeNanoseconds);
  EXPECT_EQ(result.reason, "point_time_field_extracted");
}

TEST(PointCloud2TimeFieldExtractorCoreTest, EmptyCloudDoesNotExtractWindow) {
  const std::vector<std::uint8_t> data;
  const auto view = MakeView(0, 0, sizeof(double), data);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(view, TimestampFloat64Field(0));

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.point_count_total, 0U);
  EXPECT_EQ(result.point_count_used, 0U);
  EXPECT_EQ(result.reason, "empty_cloud");
}

TEST(PointCloud2TimeFieldExtractorCoreTest, DataTooSmallDoesNotExtractWindow) {
  std::vector<std::uint8_t> data(sizeof(double));
  const auto view = MakeView(0, 3, sizeof(double), data);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(view, TimestampFloat64Field(0));

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.reason, "cloud_data_too_small");
}

TEST(PointCloud2TimeFieldExtractorCoreTest, FieldOutsidePointStepDoesNotExtractWindow) {
  std::vector<std::uint8_t> data(3 * sizeof(float));
  const auto view = MakeView(0, 3, sizeof(float), data);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(view, TimestampFloat64Field(0));

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.reason, "time_field_exceeds_point_step");
}

TEST(PointCloud2TimeFieldExtractorCoreTest, RejectsAbsoluteTimestampFloat32AsUnsafe) {
  constexpr std::uint32_t point_count = 3;
  constexpr std::uint32_t point_step = sizeof(float);

  std::vector<std::uint8_t> data(point_count * point_step);

  WriteValue<float>(&data, point_step, 0, 0, 1781901358.0F);
  WriteValue<float>(&data, point_step, 1, 0, 1781901358.05F);
  WriteValue<float>(&data, point_step, 2, 0, 1781901358.10F);

  const auto view = MakeView(0, point_count, point_step, data);

  const PointCloud2TimeFieldExtractor extractor;
  const auto result = extractor.Extract(view, TimestampFloat32Field(0));

  EXPECT_FALSE(result.has_scan_window);
  EXPECT_EQ(result.point_count_total, 3U);
  EXPECT_EQ(result.point_count_used, 0U);
  EXPECT_EQ(result.reason, "absolute_float32_timestamp_precision_unsafe");
}

}  // namespace
}  // namespace causal_slam::pointcloud
