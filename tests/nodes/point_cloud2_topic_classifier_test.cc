#include "apps/ros2/point_cloud2_topic_classifier.h"

#include <gtest/gtest.h>

namespace causal_slam::apps::ros2 {
namespace {

TEST(PointCloud2TopicClassifierTest, ClassifiesLikelyLidarTopic) {
  EXPECT_EQ(ClassifyPointCloud2TopicName("/os_cloud_node/points"), PointCloud2TopicClassification::kLikelyLidar);
}

TEST(PointCloud2TopicClassifierTest, ClassifiesProcessedCloudTopic) {
  EXPECT_EQ(ClassifyPointCloud2TopicName("/cloud_registered"), PointCloud2TopicClassification::kProcessedCloud);
}

TEST(PointCloud2TopicClassifierTest, ClassifiesSuspiciousNonLidarTopic) {
  EXPECT_EQ(ClassifyPointCloud2TopicName("/radar/cloud"), PointCloud2TopicClassification::kSuspiciousNonLidarName);
}

TEST(PointCloud2TopicClassifierTest, LeavesUnknownCloudUnclassified) {
  EXPECT_EQ(ClassifyPointCloud2TopicName("/cloud"), PointCloud2TopicClassification::kUnclassified);
}

TEST(PointCloud2TopicClassifierTest, RendersStableClassificationNames) {
  EXPECT_STREQ(ToString(PointCloud2TopicClassification::kLikelyLidar), "likely_lidar");
  EXPECT_STREQ(ToString(PointCloud2TopicClassification::kProcessedCloud), "processed_cloud");
  EXPECT_STREQ(ToString(PointCloud2TopicClassification::kSuspiciousNonLidarName), "suspicious_non_lidar_name");
  EXPECT_STREQ(ToString(PointCloud2TopicClassification::kUnclassified), "unclassified_pointcloud2");
}

}  // namespace
}  // namespace causal_slam::apps::ros2
