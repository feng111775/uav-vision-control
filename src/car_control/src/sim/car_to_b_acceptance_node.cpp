#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

class CarToBAcceptanceNode : public rclcpp::Node
{
public:
  CarToBAcceptanceNode()
  : Node("car_to_b_acceptance_node")
  {
    auto qos = rclcpp::QoS(20);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        finite_ = finite_ && std::isfinite(p.x) && std::isfinite(p.y);
        odom_received_ = true;
      });
    linear_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/control/commanded_linear_speed_mps", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->data);
        maximum_linear_ = std::max(maximum_linear_, msg->data);
        if (msg->data > 0.0) {
          minimum_positive_linear_ = std::min(minimum_positive_linear_, msg->data);
        }
        current_linear_ = msg->data;
        if (stage_ == "A_TO_B" && msg->data > 0.05) {
          moved_ = true;
        }
      });
    angular_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/control/commanded_angular_speed_radps", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->data);
        maximum_angular_ = std::max(maximum_angular_, std::abs(msg->data));
      });
    line_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/control/line_state", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data == "FAULT") {
          faulted_ = true;
        }
        if (msg->data == "RECOVERY" && previous_line_state_ != "RECOVERY") {
          ++recovery_entries_;
        }
        previous_line_state_ = msg->data;
      });
    stage_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/progress/stage", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        stage_ = msg->data;
        if (stage_ == "B_TO_C" && !b_stage_seen_) {
          b_stage_seen_ = true;
          b_stage_time_ = now();
        }
      });
    marker_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/progress/last_marker", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        last_marker_ = msg->data;
      });
    ab_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/progress/a_to_b_time_s", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->data);
        ab_time_s_ = msg->data;
      });
    fault_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/control/fault_reason", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (!msg->data.empty()) {
          faulted_ = true;
          fault_reason_ = msg->data;
        }
      });
    start_wall_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {check();});
  }

  bool success() const noexcept {return success_;}

private:
  void check()
  {
    if (faulted_ || !finite_) {
      finish(false, faulted_ ? "FAULT: " + fault_reason_ : "non-finite value");
      return;
    }
    if (b_stage_seen_ && last_marker_ == "B" && current_linear_ > 0.05 &&
      (now() - b_stage_time_).seconds() >= 1.0)
    {
      const bool passed = moved_ && odom_received_ && ab_time_s_ > 0.0 && ab_time_s_ < 15.0;
      finish(passed, passed ? "B accepted and motion continued" : "B metrics invalid");
      return;
    }
    if (std::chrono::steady_clock::now() - start_wall_ > std::chrono::seconds(30)) {
      finish(false, "wall-clock timeout before B");
    }
  }

  void finish(bool passed, const std::string & reason)
  {
    success_ = passed;
    RCLCPP_INFO(
      get_logger(),
      "%s: B=%s A-B=%.3fs min_v=%.3f max_v=%.3f max_w=%.3f recovery_entries=%d "
      "fault=%s final_stage=%s reason=%s",
      passed ? "PASS" : "FAIL", last_marker_.c_str(), ab_time_s_,
      std::isfinite(minimum_positive_linear_) ? minimum_positive_linear_ : 0.0,
      maximum_linear_, maximum_angular_, recovery_entries_,
      faulted_ ? "yes" : "no", stage_.c_str(), reason.c_str());
    rclcpp::shutdown();
  }

  bool success_{false};
  bool finite_{true};
  bool faulted_{false};
  bool moved_{false};
  bool odom_received_{false};
  bool b_stage_seen_{false};
  double current_linear_{0.0};
  double minimum_positive_linear_{std::numeric_limits<double>::infinity()};
  double maximum_linear_{0.0};
  double maximum_angular_{0.0};
  double ab_time_s_{0.0};
  int recovery_entries_{0};
  std::string stage_{"WAIT_START"};
  std::string last_marker_{"NONE"};
  std::string previous_line_state_;
  std::string fault_reason_;
  rclcpp::Time b_stage_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point start_wall_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angular_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr line_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ab_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarToBAcceptanceNode>();
  rclcpp::spin(node);
  return node->success() ? 0 : 1;
}
