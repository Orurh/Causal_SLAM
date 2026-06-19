#include <algorithm>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace causal_slam::nodes {

using namespace std::chrono_literals;

class FakeImuPublisherNode final : public rclcpp::Node {
 public:
  explicit FakeImuPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
      : rclcpp::Node("fake_imu_publisher_node", options) {
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS{});

    const double period_ms = this->declare_parameter<double>("period_ms", 10.0);
    const double safe_period_ms = std::max(period_ms, 1.0);

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_period_ms)),
        [this]() { PublishImu(); });

    RCLCPP_INFO(this->get_logger(), "FakeImuPublisherNode started | period_ms=%.3f", safe_period_ms);
  }

 private:
  void PublishImu() {
    sensor_msgs::msg::Imu msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    msg.orientation.w = 1.0;

    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.0;

    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 9.81;

    imu_publisher_->publish(msg);

    ++published_count_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::uint64_t published_count_{0};
};

}  // namespace causal_slam::nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<causal_slam::nodes::FakeImuPublisherNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}