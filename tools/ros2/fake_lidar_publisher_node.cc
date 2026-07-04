#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "adapters/ros2/support/point_cloud_qos.h"

namespace causal_slam::nodes {
namespace {

using SteadyClock = std::chrono::steady_clock;

std::string NormalizeTimeFieldMode(std::string value) {
  std::ranges::transform(value, value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

  return value;
}

sensor_msgs::msg::PointField MakeField(std::string name, std::uint32_t offset, std::uint8_t datatype, std::uint32_t count = 1) {
  sensor_msgs::msg::PointField field;
  field.name = std::move(name);
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

template <typename T>
void WritePointFieldValue(sensor_msgs::msg::PointCloud2& msg, std::uint32_t point_index, std::uint32_t field_offset, const T& value) {
  const std::size_t byte_offset = static_cast<std::size_t>(point_index) * msg.point_step + field_offset;

  std::memcpy(msg.data.data() + byte_offset, &value, sizeof(T));
}

std::int64_t MillisecondsToNanoseconds(double milliseconds) {
  constexpr double nanoseconds_per_millisecond = 1'000'000.0;
  return static_cast<std::int64_t>(milliseconds * nanoseconds_per_millisecond);
}

double DurationUs(SteadyClock::duration duration) {
  return std::chrono::duration<double, std::micro>(duration).count();
}

}  // namespace

class FakeLidarPublisherNode final : public rclcpp::Node {
 public:
  explicit FakeLidarPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
      : rclcpp::Node("fake_lidar_publisher_node", options) {
    const std::string lidar_topic = this->declare_parameter<std::string>("lidar_topic", "/points");

    frame_id_ = this->declare_parameter<std::string>("frame_id", "lidar");

    const double period_ms = this->declare_parameter<double>("period_ms", 100.0);
    const double safe_period_ms = std::max(period_ms, 1.0);

    const int point_count = this->declare_parameter<int>("point_count", 0);
    point_count_ = static_cast<std::uint32_t>(std::max(point_count, 0));

    include_xyz_fields_ = this->declare_parameter<bool>("include_xyz_fields", false);

    time_field_mode_ = NormalizeTimeFieldMode(this->declare_parameter<std::string>("time_field_mode", "none"));

    const double scan_duration_ms = this->declare_parameter<double>("scan_duration_ms", safe_period_ms);
    scan_duration_ms_ = std::max(scan_duration_ms, 0.0);

    perf_metrics_enabled_ = this->declare_parameter<bool>("perf_metrics_enabled", false);
    const double perf_metrics_period_ms = this->declare_parameter<double>("perf_metrics_period_ms", 2000.0);
    const double safe_perf_metrics_period_ms = std::max(perf_metrics_period_ms, 100.0);

    const std::string qos_reliability = this->declare_parameter<std::string>("qos_reliability", "best_effort");
    const int qos_depth = std::max(static_cast<int>(this->declare_parameter<int>("qos_depth", 5)), 1);

    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        lidar_topic, causal_slam::ros_support::MakePointCloudQos(qos_reliability, qos_depth));

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_period_ms)),
        [this]() { PublishCloud(); });

    if (perf_metrics_enabled_) {
      perf_metrics_timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(safe_perf_metrics_period_ms)),
          [this]() { LogPerfMetrics(); });
    }

    RCLCPP_INFO(this->get_logger(),
                "FakeLidarPublisherNode started"
                " | lidar_topic=%s"
                " | frame_id=%s"
                " | period_ms=%.3f"
                " | point_count=%u"
                " | include_xyz_fields=%s"
                " | time_field_mode=%s"
                " | scan_duration_ms=%.3f"
                " | perf_metrics_enabled=%s"
                " | qos_reliability=%s"
                " | qos_depth=%d",
                lidar_topic.c_str(), frame_id_.c_str(), safe_period_ms, point_count_, include_xyz_fields_ ? "true" : "false",
                time_field_mode_.c_str(), scan_duration_ms_, perf_metrics_enabled_ ? "true" : "false", qos_reliability.c_str(), qos_depth);
  }

 private:
  void PublishCloud() {
    const auto callback_started_at = SteadyClock::now();

    std::optional<double> actual_period_us;
    if (last_publish_started_at_.has_value()) {
      actual_period_us = DurationUs(callback_started_at - *last_publish_started_at_);
    }
    last_publish_started_at_ = callback_started_at;

    sensor_msgs::msg::PointCloud2 msg;

    const rclcpp::Time now = this->now();
    const std::int64_t now_ns = now.nanoseconds();

    std::int64_t header_stamp_ns = now_ns;

    if (time_field_mode_ == "offset_time_uint32") {
      header_stamp_ns = now_ns - MillisecondsToNanoseconds(scan_duration_ms_);
    }

    msg.header.stamp = rclcpp::Time(header_stamp_ns);
    msg.header.frame_id = frame_id_;

    msg.height = 1;
    msg.width = point_count_;
    msg.is_bigendian = false;
    msg.is_dense = true;

    const auto generation_started_at = SteadyClock::now();
    ConfigureFields(msg);
    ConfigureData(now_ns, msg);
    const auto generation_finished_at = SteadyClock::now();

    const auto publish_started_at = SteadyClock::now();
    lidar_publisher_->publish(msg);
    const auto publish_finished_at = SteadyClock::now();

    ++published_count_;

    RecordPerfMetrics(DurationUs(generation_finished_at - generation_started_at), DurationUs(publish_finished_at - publish_started_at),
                      actual_period_us, msg.data.size());
  }

  void RecordPerfMetrics(double generation_us, double publish_call_us, std::optional<double> actual_period_us,
                         std::size_t data_size_bytes) {
    if (!perf_metrics_enabled_) {
      return;
    }

    ++perf_window_count_;
    perf_window_bytes_ += static_cast<std::uint64_t>(data_size_bytes);

    perf_generation_total_us_ += generation_us;
    perf_generation_max_us_ = std::max(perf_generation_max_us_, generation_us);

    perf_publish_total_us_ += publish_call_us;
    perf_publish_max_us_ = std::max(perf_publish_max_us_, publish_call_us);

    if (actual_period_us.has_value()) {
      ++perf_period_count_;
      perf_period_total_us_ += *actual_period_us;
      perf_period_max_us_ = std::max(perf_period_max_us_, *actual_period_us);
    }
  }

  void LogPerfMetrics() {
    if (!perf_metrics_enabled_) {
      return;
    }

    if (perf_window_count_ == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "FakeLidarPublisher metrics | published_count=%lu | "
                  "window_count=0",
                  published_count_);
      return;
    }

    const double count = static_cast<double>(perf_window_count_);
    const double period_count = std::max<double>(static_cast<double>(perf_period_count_), 1.0);
    const double mib = static_cast<double>(perf_window_bytes_) / 1024.0 / 1024.0;

    RCLCPP_INFO(this->get_logger(),
                "FakeLidarPublisher metrics | published_count=%lu | window_count=%lu | "
                "window_mib=%.3f | generation_avg_ms=%.3f | generation_max_ms=%.3f | "
                "publish_call_avg_ms=%.3f | publish_call_max_ms=%.3f | "
                "actual_period_avg_ms=%.3f | actual_period_max_ms=%.3f",
                published_count_, perf_window_count_, mib, perf_generation_total_us_ / count / 1000.0, perf_generation_max_us_ / 1000.0,
                perf_publish_total_us_ / count / 1000.0, perf_publish_max_us_ / 1000.0, perf_period_total_us_ / period_count / 1000.0,
                perf_period_max_us_ / 1000.0);

    perf_window_count_ = 0;
    perf_window_bytes_ = 0;
    perf_period_count_ = 0;
    perf_generation_total_us_ = 0.0;
    perf_generation_max_us_ = 0.0;
    perf_publish_total_us_ = 0.0;
    perf_publish_max_us_ = 0.0;
    perf_period_total_us_ = 0.0;
    perf_period_max_us_ = 0.0;
  }

  void ConfigureFields(sensor_msgs::msg::PointCloud2& msg) const {
    std::uint32_t offset = 0;

    if (include_xyz_fields_) {
      msg.fields.push_back(MakeField("x", offset, sensor_msgs::msg::PointField::FLOAT32));
      offset += sizeof(float);

      msg.fields.push_back(MakeField("y", offset, sensor_msgs::msg::PointField::FLOAT32));
      offset += sizeof(float);

      msg.fields.push_back(MakeField("z", offset, sensor_msgs::msg::PointField::FLOAT32));
      offset += sizeof(float);
    }

    if (time_field_mode_ == "timestamp_float64") {
      msg.fields.push_back(MakeField("timestamp", offset, sensor_msgs::msg::PointField::FLOAT64));
      offset += sizeof(double);
    } else if (time_field_mode_ == "timestamp_float32") {
      msg.fields.push_back(MakeField("timestamp", offset, sensor_msgs::msg::PointField::FLOAT32));
      offset += sizeof(float);
    } else if (time_field_mode_ == "offset_time_uint32") {
      msg.fields.push_back(MakeField("offset_time", offset, sensor_msgs::msg::PointField::UINT32));
      offset += sizeof(std::uint32_t);
    } else if (time_field_mode_ == "split_time_uint32") {
      msg.fields.push_back(MakeField("timeSecond", offset, sensor_msgs::msg::PointField::UINT32));
      offset += sizeof(std::uint32_t);

      msg.fields.push_back(MakeField("timeNanosecond", offset, sensor_msgs::msg::PointField::UINT32));
      offset += sizeof(std::uint32_t);
    }

    msg.point_step = offset;
    msg.row_step = msg.point_step * msg.width;
  }

  void ConfigureData(std::int64_t scan_end_ns, sensor_msgs::msg::PointCloud2& msg) const {
    msg.data.resize(msg.row_step);

    if (msg.width == 0 || msg.point_step == 0) {
      return;
    }

    const std::int64_t scan_duration_ns = MillisecondsToNanoseconds(scan_duration_ms_);
    const std::int64_t scan_start_ns = scan_end_ns - scan_duration_ns;

    for (std::uint32_t point_index = 0; point_index < msg.width; ++point_index) {
      const double ratio = msg.width == 1 ? 0.0 : static_cast<double>(point_index) / static_cast<double>(msg.width - 1);

      const std::int64_t point_offset_ns = static_cast<std::int64_t>(ratio * static_cast<double>(scan_duration_ns));
      const std::int64_t point_stamp_ns = scan_start_ns + point_offset_ns;

      WriteXyzFields(point_index, msg);
      WriteTimeFields(point_index, point_offset_ns, point_stamp_ns, msg);
    }
  }

  void WriteXyzFields(std::uint32_t point_index, sensor_msgs::msg::PointCloud2& msg) const {
    if (!include_xyz_fields_) {
      return;
    }

    const float x = static_cast<float>(point_index);
    const float y = 0.0F;
    const float z = 0.0F;

    WritePointFieldValue(msg, point_index, 0, x);
    WritePointFieldValue(msg, point_index, sizeof(float), y);
    WritePointFieldValue(msg, point_index, sizeof(float) * 2, z);
  }

  void WriteTimeFields(std::uint32_t point_index, std::int64_t point_offset_ns, std::int64_t point_stamp_ns,
                       sensor_msgs::msg::PointCloud2& msg) const {
    const std::uint32_t time_offset = include_xyz_fields_ ? static_cast<std::uint32_t>(sizeof(float) * 3) : 0U;

    if (time_field_mode_ == "timestamp_float64") {
      constexpr double nanoseconds_per_second = 1'000'000'000.0;
      const double timestamp_seconds = static_cast<double>(point_stamp_ns) / nanoseconds_per_second;

      WritePointFieldValue(msg, point_index, time_offset, timestamp_seconds);
      return;
    }

    if (time_field_mode_ == "timestamp_float32") {
      constexpr float nanoseconds_per_second = 1'000'000'000.0F;
      const float timestamp_seconds = static_cast<float>(point_stamp_ns) / nanoseconds_per_second;

      WritePointFieldValue(msg, point_index, time_offset, timestamp_seconds);
      return;
    }

    if (time_field_mode_ == "offset_time_uint32") {
      const std::uint32_t offset_time_ns = static_cast<std::uint32_t>(std::max<std::int64_t>(point_offset_ns, 0));

      WritePointFieldValue(msg, point_index, time_offset, offset_time_ns);
      return;
    }

    if (time_field_mode_ == "split_time_uint32") {
      constexpr std::int64_t nanoseconds_per_second = 1'000'000'000LL;

      const std::uint32_t seconds = static_cast<std::uint32_t>(point_stamp_ns / nanoseconds_per_second);
      const std::uint32_t nanoseconds = static_cast<std::uint32_t>(point_stamp_ns % nanoseconds_per_second);

      WritePointFieldValue(msg, point_index, time_offset, seconds);
      WritePointFieldValue(msg, point_index, time_offset + static_cast<std::uint32_t>(sizeof(std::uint32_t)), nanoseconds);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr perf_metrics_timer_;

  std::string frame_id_{"lidar"};
  std::string time_field_mode_{"none"};

  std::uint32_t point_count_{0};
  bool include_xyz_fields_{false};
  double scan_duration_ms_{100.0};

  std::uint64_t published_count_{0};

  bool perf_metrics_enabled_{false};
  std::optional<SteadyClock::time_point> last_publish_started_at_;

  std::uint64_t perf_window_count_{0};
  std::uint64_t perf_window_bytes_{0};
  std::uint64_t perf_period_count_{0};

  double perf_generation_total_us_{0.0};
  double perf_generation_max_us_{0.0};
  double perf_publish_total_us_{0.0};
  double perf_publish_max_us_{0.0};
  double perf_period_total_us_{0.0};
  double perf_period_max_us_{0.0};
};

}  // namespace causal_slam::nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<causal_slam::nodes::FakeLidarPublisherNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}