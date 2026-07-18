#include "apps/ros2/custom_sensor_topic_classifier.h"

#include <gtest/gtest.h>

namespace causal_slam::apps::ros2 {
namespace {

TEST(CustomSensorTopicClassifierTest, RecognizesKnownCustomTypes) {
  EXPECT_TRUE(IsKnownCustomSensorType("livox_ros_driver/msg/CustomMsg"));
  EXPECT_TRUE(IsKnownCustomSensorType("livox_ros_driver2/msg/CustomMsg"));
  EXPECT_TRUE(IsKnownCustomSensorType("ouster_ros/msg/PacketMsg"));

  EXPECT_FALSE(IsKnownCustomSensorType("sensor_msgs/msg/PointCloud2"));
}

TEST(CustomSensorTopicClassifierTest, ClassifiesLivoxCustomMessage) {
  EXPECT_EQ(ClassifyCustomSensorTopic("/livox/avia/lidar", "livox_ros_driver/msg/CustomMsg"),
            CustomSensorTopicClassification::kCustomLidar);

  EXPECT_EQ(ClassifyCustomSensorTopic("/livox/mid360/lidar", "livox_ros_driver2/msg/CustomMsg"),
            CustomSensorTopicClassification::kCustomLidar);
}

TEST(CustomSensorTopicClassifierTest, ClassifiesOusterLidarPackets) {
  EXPECT_EQ(ClassifyCustomSensorTopic("/os_cloud_node/lidar_packets", "ouster_ros/msg/PacketMsg"),
            CustomSensorTopicClassification::kRawLidarPackets);
}

TEST(CustomSensorTopicClassifierTest, ClassifiesOusterImuPackets) {
  EXPECT_EQ(ClassifyCustomSensorTopic("/os_cloud_node/imu_packets", "ouster_ros/msg/PacketMsg"),
            CustomSensorTopicClassification::kRawImuPackets);
}

TEST(CustomSensorTopicClassifierTest, KeepsUnknownOusterPacketsVisible) {
  EXPECT_EQ(ClassifyCustomSensorTopic("/ouster/packets", "ouster_ros/msg/PacketMsg"), CustomSensorTopicClassification::kRawSensorPackets);
}

TEST(CustomSensorTopicClassifierTest, RejectsUnrelatedTypes) {
  EXPECT_EQ(ClassifyCustomSensorTopic("/points", "sensor_msgs/msg/PointCloud2"), CustomSensorTopicClassification::kNotCustomSensorType);
}

TEST(CustomSensorTopicClassifierTest, RendersStableNames) {
  EXPECT_STREQ(ToString(CustomSensorTopicClassification::kCustomLidar), "custom_lidar");
  EXPECT_STREQ(ToString(CustomSensorTopicClassification::kRawLidarPackets), "raw_lidar_packets");
  EXPECT_STREQ(ToString(CustomSensorTopicClassification::kRawImuPackets), "raw_imu_packets");
  EXPECT_STREQ(ToString(CustomSensorTopicClassification::kRawSensorPackets), "raw_sensor_packets");
}

}  // namespace
}  // namespace causal_slam::apps::ros2
