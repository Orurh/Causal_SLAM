#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace causal_slam::nodes {
namespace {

std::int64_t MillisecondsToNanoseconds(double milliseconds) {
  constexpr double kNanosecondsPerMillisecond = 1'000'000.0;
  return static_cast<std::int64_t>(milliseconds * kNanosecondsPerMillisecond);
}

}  // namespace

using namespace std::chrono_literals;

class FakeImuPublisherNode final : public rclcpp::Node {
 public:
  explicit FakeImuPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
      : rclcpp::Node("fake_imu_publisher_node", options) {
    const std::string imu_topic = this->declare_parameter<std::string>("imu_topic", "/imu/data");

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS{});

    const double period_ms = this->declare_parameter<double>("period_ms", 10.0);
    const double safe_period_ms = std::max(period_ms, 1.0);

    const double timestamp_shift_ms = this->declare_parameter<double>("timestamp_shift_ms", 0.0);
    timestamp_shift_ns_ = MillisecondsToNanoseconds(timestamp_shift_ms);

    const int drop_every_n = this->declare_parameter<int>("drop_every_n", 0);
    drop_every_n_ = static_cast<std::uint64_t>(std::max(drop_every_n, 0));

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_period_ms)),
        [this]() { PublishImu(); });

    RCLCPP_INFO(this->get_logger(),
                "FakeImuPublisherNode started"
                " | imu_topic=%s"
                " | period_ms=%.3f"
                " | timestamp_shift_ms=%.3f"
                " | drop_every_n=%lu",
                imu_topic.c_str(), safe_period_ms, timestamp_shift_ms, static_cast<unsigned long>(drop_every_n_));
  }

 private:
  void PublishImu() {
    ++publish_attempt_count_;

    if (drop_every_n_ > 0 && publish_attempt_count_ % drop_every_n_ == 0) {
      return;
    }

    sensor_msgs::msg::Imu msg;

    const std::int64_t stamp_ns = this->now().nanoseconds() + timestamp_shift_ns_;
    msg.header.stamp = rclcpp::Time(stamp_ns);
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

  std::int64_t timestamp_shift_ns_{0};
  std::uint64_t drop_every_n_{0};
  std::uint64_t publish_attempt_count_{0};

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