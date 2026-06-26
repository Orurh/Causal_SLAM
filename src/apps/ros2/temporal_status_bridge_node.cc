#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace causal_slam::nodes {
namespace {

using namespace std::chrono_literals;

diagnostic_msgs::msg::KeyValue MakeKeyValue(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = std::move(key);
  kv.value = std::move(value);
  return kv;
}

std::string BoolToString(bool value) {
  return value ? "true" : "false";
}

}  // namespace

class TemporalStatusBridgeNode final : public rclcpp::Node {
 public:
  explicit TemporalStatusBridgeNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
      : rclcpp::Node("temporal_status_bridge_node", options) {
    const std::string map_update_allowed_topic =
        this->declare_parameter<std::string>(
            "map_update_allowed_topic", "/causal_slam/map_update_allowed");
    const std::string temporal_health_topic =
        this->declare_parameter<std::string>(
            "temporal_health_topic", "/causal_slam/temporal_health");
    const std::string map_update_reason_topic =
        this->declare_parameter<std::string>(
            "map_update_reason_topic", "/causal_slam/map_update_reason");
    const std::string fault_reasons_topic =
        this->declare_parameter<std::string>(
            "fault_reasons_topic", "/causal_slam/fault_reasons");
    const std::string decision_json_topic =
        this->declare_parameter<std::string>(
            "map_update_decision_json_topic",
            "/causal_slam/map_update_decision_json");
    const std::string diagnostics_topic =
        this->declare_parameter<std::string>("diagnostics_topic",
                                             "/diagnostics");

    diagnostics_publisher_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic, rclcpp::SystemDefaultsQoS{});

    allowed_subscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            map_update_allowed_topic, rclcpp::SystemDefaultsQoS{},
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
              latest_allowed_ = msg->data;
              has_allowed_ = true;
            });

    health_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            temporal_health_topic, rclcpp::SystemDefaultsQoS{},
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
              latest_health_ = msg->data;
              has_health_ = true;
            });

    reason_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            map_update_reason_topic, rclcpp::SystemDefaultsQoS{},
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
              latest_reason_ = msg->data;
            });

    fault_reasons_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            fault_reasons_topic, rclcpp::SystemDefaultsQoS{},
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
              latest_fault_reasons_ = msg->data;
            });

    decision_json_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            decision_json_topic, rclcpp::SystemDefaultsQoS{},
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
              latest_decision_json_ = msg->data;
            });

    timer_ = this->create_wall_timer(500ms, [this] { PublishDiagnostics(); });

    RCLCPP_INFO(this->get_logger(),
                "TemporalStatusBridgeNode started"
                " | diagnostics_topic=%s"
                " | allowed_topic=%s"
                " | health_topic=%s"
                " | fault_reasons_topic=%s",
                diagnostics_topic.c_str(),
                map_update_allowed_topic.c_str(),
                temporal_health_topic.c_str(),
                fault_reasons_topic.c_str());
  }

 private:
  void PublishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "causal_slam/temporal_gate";
    status.hardware_id = "causal_slam";

    if (!has_allowed_ || !has_health_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "waiting_for_temporal_gate_status";
    } else if (latest_health_ == "OK" && latest_allowed_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "map_update_allowed";
    } else if (latest_health_ == "WARNING" && latest_allowed_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "map_update_allowed_with_warning";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "map_update_blocked";
    }

    status.values.push_back(
        MakeKeyValue("map_update_allowed", BoolToString(latest_allowed_)));
    status.values.push_back(MakeKeyValue("temporal_health", latest_health_));
    status.values.push_back(MakeKeyValue("map_update_reason", latest_reason_));
    status.values.push_back(MakeKeyValue("fault_reasons", latest_fault_reasons_));
    status.values.push_back(
        MakeKeyValue("has_decision_json", BoolToString(!latest_decision_json_.empty())));

    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allowed_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reason_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_reasons_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_json_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_allowed_{false};
  bool has_health_{false};
  bool latest_allowed_{false};
  std::string latest_health_;
  std::string latest_reason_;
  std::string latest_fault_reasons_;
  std::string latest_decision_json_;
};

}  // namespace causal_slam::nodes

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<causal_slam::nodes::TemporalStatusBridgeNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
