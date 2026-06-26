#pragma once

#include <cstdint>

namespace causal_slam::pointcloud {

// Values intentionally match sensor_msgs/PointField datatype constants.
// Keeping them ROS-free allows both ROS2 and ROS1 adapters to reuse the same
// PointCloud2 field/time inspection logic.
inline constexpr std::uint8_t kPointCloud2Int8 = 1;
inline constexpr std::uint8_t kPointCloud2Uint8 = 2;
inline constexpr std::uint8_t kPointCloud2Int16 = 3;
inline constexpr std::uint8_t kPointCloud2Uint16 = 4;
inline constexpr std::uint8_t kPointCloud2Int32 = 5;
inline constexpr std::uint8_t kPointCloud2Uint32 = 6;
inline constexpr std::uint8_t kPointCloud2Float32 = 7;
inline constexpr std::uint8_t kPointCloud2Float64 = 8;

}  // namespace causal_slam::pointcloud
