#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "temporal_monitor_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<causal_slam::nodes::TemporalMonitorNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}