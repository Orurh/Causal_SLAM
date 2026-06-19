#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace causal_slam::nodes {

class FakeLidarPublisherNode final : public rclcpp::Node {
 public:
  explicit FakeLidarPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
      : rclcpp::Node("fake_lidar_publisher_node", options) {
    const std::string lidar_topic = this->declare_parameter<std::string>("lidar_topic", "/points");

    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SensorDataQoS{});

    const double period_ms = this->declare_parameter<double>("period_ms", 100.0);
    const double safe_period_ms = std::max(period_ms, 1.0);

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_period_ms)),
        [this]() { PublishCloud(); });

    RCLCPP_INFO(this->get_logger(), "FakeLidarPublisherNode started | lidar_topic=%s | period_ms=%.3f", lidar_topic.c_str(),
                safe_period_ms);
  }

 private:
  void PublishCloud() {
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "lidar";

    msg.height = 1;
    msg.width = 0;
    msg.is_bigendian = false;
    msg.point_step = 0;
    msg.row_step = 0;
    msg.is_dense = true;

    lidar_publisher_->publish(msg);

    ++published_count_;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::uint64_t published_count_{0};
};

}  // namespace causal_slam::nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<causal_slam::nodes::FakeLidarPublisherNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}