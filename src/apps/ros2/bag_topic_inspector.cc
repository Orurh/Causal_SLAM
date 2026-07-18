#include "apps/ros2/bag_topic_inspector.h"

#include <map>

#include <rosbag2_cpp/reader.hpp>

namespace causal_slam::apps::ros2 {

BagTopicInspection InspectBagTopics(const std::string& bag_path) {
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);

  const auto topics = reader.get_all_topics_and_types();

  std::map<std::string, std::uint64_t> message_counts;
  for (const auto& topic : topics) {
    message_counts[topic.name] = 0;
  }

  while (reader.has_next()) {
    const auto message = reader.read_next();
    ++message_counts[message->topic_name];
  }

  BagTopicInspection inspection;
  inspection.bag_path = bag_path;
  inspection.topics.reserve(topics.size());

  for (const auto& topic : topics) {
    BagTopicInfo info;
    info.name = topic.name;
    info.type = topic.type;

    if (const auto it = message_counts.find(topic.name); it != message_counts.end()) {
      info.message_count = it->second;
    }

    inspection.topics.push_back(std::move(info));
  }

  return inspection;
}

}  // namespace causal_slam::apps::ros2
