#include <algorithm>
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

class ImuFaultInjectionNode final : public rclcpp::Node {
 public:
  explicit ImuFaultInjectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
      : rclcpp::Node("imu_fault_injection_node", options) {
    const std::string input_topic = this->declare_parameter<std::string>("input_topic", "/imu/raw");
    const std::string output_topic = this->declare_parameter<std::string>("output_topic", "/imu/faulty");

    const double timestamp_shift_ms = this->declare_parameter<double>("timestamp_shift_ms", 0.0);
    timestamp_shift_ns_ = MillisecondsToNanoseconds(timestamp_shift_ms);

    const int drop_every_n = this->declare_parameter<int>("drop_every_n", 0);
    drop_every_n_ = static_cast<std::uint64_t>(std::max(drop_every_n, 0));

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic, rclcpp::SensorDataQoS{});

    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(input_topic, rclcpp::SensorDataQoS{},
                                                                     [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(msg); });

    RCLCPP_INFO(this->get_logger(),
                "ImuFaultInjectionNode started"
                " | input_topic=%s"
                " | output_topic=%s"
                " | timestamp_shift_ms=%.3f"
                " | drop_every_n=%lu",
                input_topic.c_str(), output_topic.c_str(), timestamp_shift_ms, static_cast<unsigned long>(drop_every_n_));
  }

 private:
  void OnImu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    ++received_count_;

    if (drop_every_n_ > 0 && received_count_ % drop_every_n_ == 0) {
      return;
    }

    auto output = *msg;

    const std::int64_t shifted_stamp_ns = rclcpp::Time(output.header.stamp).nanoseconds() + timestamp_shift_ns_;
    output.header.stamp = rclcpp::Time(shifted_stamp_ns);

    publisher_->publish(output);
    ++published_count_;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

  std::int64_t timestamp_shift_ns_{0};
  std::uint64_t drop_every_n_{0};
  std::uint64_t received_count_{0};
  std::uint64_t published_count_{0};
};

}  // namespace causal_slam::nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<causal_slam::nodes::ImuFaultInjectionNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
