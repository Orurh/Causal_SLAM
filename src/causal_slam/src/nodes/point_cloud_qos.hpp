#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <string>

#include <rclcpp/qos.hpp>

namespace causal_slam::nodes {

inline std::string NormalizeQosReliability(std::string value) {
  std::ranges::transform(value, value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

inline rclcpp::QoS MakePointCloudQos(const std::string& reliability, int depth) {
  const int safe_depth = std::max(depth, 1);
  const std::string normalized = NormalizeQosReliability(reliability);

  rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(safe_depth))};

  if (normalized == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }

  qos.durability_volatile();
  return qos;
}

}  // namespace causal_slam::nodes
