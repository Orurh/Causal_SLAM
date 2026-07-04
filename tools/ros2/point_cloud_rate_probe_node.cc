#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adapters/ros2/support/point_cloud_qos.h"

namespace {

using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using SteadyClock = std::chrono::steady_clock;

double ToSeconds(SteadyClock::duration duration) {
  return std::chrono::duration<double>(duration).count();
}

double ToMilliseconds(double microseconds) {
  return microseconds / 1000.0;
}

double Percentile(std::vector<double> values, double q) {
  if (values.empty()) {
    return 0.0;
  }

  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1));
  return values[index];
}

class PointCloudRateProbeNode final : public rclcpp::Node {
 public:
  PointCloudRateProbeNode() : Node("point_cloud_rate_probe_node") {
    topic_ = declare_parameter<std::string>("topic", "/points_raw");
    label_ = declare_parameter<std::string>("label", "pointcloud");
    const double summary_period_ms = declare_parameter<double>("summary_period_ms", 2000.0);
    const std::string qos_reliability = declare_parameter<std::string>("qos_reliability", "best_effort");
    const int qos_depth = std::max(static_cast<int>(declare_parameter<int>("qos_depth", 5)), 1);

    started_at_ = SteadyClock::now();
    last_summary_at_ = started_at_;

    subscription_ = create_subscription<PointCloud2Msg>(topic_, causal_slam::ros_support::MakePointCloudQos(qos_reliability, qos_depth),
                                                        [this](PointCloud2Msg::ConstSharedPtr msg) { OnCloud(msg); });

    timer_ =
        create_wall_timer(std::chrono::milliseconds(static_cast<int>(std::max(summary_period_ms, 100.0))), [this]() { PrintSummary(); });

    RCLCPP_INFO(get_logger(),
                "PointCloudRateProbe started | label=%s | topic=%s | "
                "qos_reliability=%s | qos_depth=%d",
                label_.c_str(), topic_.c_str(), qos_reliability.c_str(), qos_depth);
  }

 private:
  void OnCloud(const PointCloud2Msg::ConstSharedPtr& msg) {
    const auto now = SteadyClock::now();

    if (last_receive_at_.has_value()) {
      const auto interval_us = std::chrono::duration<double, std::micro>(now - *last_receive_at_).count();
      window_intervals_us_.push_back(interval_us);
    }

    last_receive_at_ = now;

    const std::uint64_t data_bytes = static_cast<std::uint64_t>(msg->data.size());

    ++total_count_;
    ++window_count_;

    total_bytes_ += data_bytes;
    window_bytes_ += data_bytes;
  }

  void PrintSummary() {
    const auto now = SteadyClock::now();

    const double total_seconds = std::max(ToSeconds(now - started_at_), 1e-9);
    const double window_seconds = std::max(ToSeconds(now - last_summary_at_), 1e-9);

    const double total_hz = static_cast<double>(total_count_) / total_seconds;
    const double window_hz = static_cast<double>(window_count_) / window_seconds;

    const double window_mib = static_cast<double>(window_bytes_) / 1024.0 / 1024.0;
    const double window_mib_s = window_mib / window_seconds;

    const double avg_interval_us = window_intervals_us_.empty()
                                       ? 0.0
                                       : std::accumulate(window_intervals_us_.begin(), window_intervals_us_.end(), 0.0) /
                                             static_cast<double>(window_intervals_us_.size());

    const double p50_interval_us = Percentile(window_intervals_us_, 0.50);
    const double p95_interval_us = Percentile(window_intervals_us_, 0.95);
    const double max_interval_us =
        window_intervals_us_.empty() ? 0.0 : *std::max_element(window_intervals_us_.begin(), window_intervals_us_.end());

    RCLCPP_INFO(get_logger(),
                "PointCloudRateProbe | label=%s | topic=%s | total_count=%lu | "
                "total_hz=%.3f | window_count=%lu | window_hz=%.3f | "
                "window_mib_s=%.3f | avg_interval_ms=%.3f | p50_interval_ms=%.3f | "
                "p95_interval_ms=%.3f | max_interval_ms=%.3f",
                label_.c_str(), topic_.c_str(), total_count_, total_hz, window_count_, window_hz, window_mib_s,
                ToMilliseconds(avg_interval_us), ToMilliseconds(p50_interval_us), ToMilliseconds(p95_interval_us),
                ToMilliseconds(max_interval_us));

    window_count_ = 0;
    window_bytes_ = 0;
    window_intervals_us_.clear();
    last_summary_at_ = now;
  }

  std::string topic_;
  std::string label_;

  rclcpp::Subscription<PointCloud2Msg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  SteadyClock::time_point started_at_;
  SteadyClock::time_point last_summary_at_;
  std::optional<SteadyClock::time_point> last_receive_at_;

  std::uint64_t total_count_{0};
  std::uint64_t total_bytes_{0};

  std::uint64_t window_count_{0};
  std::uint64_t window_bytes_{0};
  std::vector<double> window_intervals_us_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudRateProbeNode>());
  rclcpp::shutdown();
  return 0;
}
