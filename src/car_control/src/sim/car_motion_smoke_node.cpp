#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class CarMotionSmokeNode final : public rclcpp::Node
{
public:
  explicit CarMotionSmokeNode(const rclcpp::NodeOptions & options)
  : Node("car_motion_smoke_node", options), start_(std::chrono::steady_clock::now())
  {
    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_controller/cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { receive_odom(*msg); });
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }

  int result() const {return result_;}

private:
  static double yaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static bool finite(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose;
    const auto & t = msg.twist.twist;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
           std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w) &&
           std::isfinite(t.linear.x) && std::isfinite(t.angular.z);
  }

  void receive_odom(const nav_msgs::msg::Odometry & msg)
  {
    if (!finite(msg)) {
      fail("Odometry contains NaN or Inf");
      return;
    }
    latest_ = msg;
    have_odom_ = true;
    if (!have_initial_) {
      initial_ = msg;
      have_initial_ = true;
    }
  }

  void publish(double linear, double angular)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_footprint";
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    command_pub_->publish(msg);
  }

  void fail(const std::string & reason)
  {
    if (done_) {return;}
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    publish(0.0, 0.0);
    result_ = 1;
    done_ = true;
    rclcpp::shutdown();
  }

  void tick()
  {
    if (done_) {return;}
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    if (elapsed > 15.0) {
      fail("Smoke test timed out");
      return;
    }
    if (!have_odom_) {
      if (elapsed > 5.0) {fail("No odometry received within 5 seconds");}
      return;
    }

    const double phase_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - phase_start_).count();
    if (phase_ == 0) {
      if (!topic_checked_) {
        const auto infos = get_subscriptions_info_by_topic("/diff_drive_controller/cmd_vel");
        const bool correct = std::any_of(infos.begin(), infos.end(), [](const auto & info) {
          return info.topic_type() == "geometry_msgs/msg/TwistStamped";
        });
        if (!correct) {
          if (elapsed > 7.0) {
            fail("cmd_vel has no geometry_msgs/msg/TwistStamped subscriber");
          }
          return;
        }
        topic_checked_ = true;
        phase_start_ = std::chrono::steady_clock::now();
      }
      publish(0.3, 0.0);
      if (phase_time >= 2.5) {
        straight_end_ = latest_;
        const double dx = straight_end_.pose.pose.position.x - initial_.pose.pose.position.x;
        const double dy = straight_end_.pose.pose.position.y - initial_.pose.pose.position.y;
        if (std::hypot(dx, dy) < 0.35) {
          fail("Forward displacement is below 0.35 m");
          return;
        }
        phase_ = 1;
        phase_start_ = std::chrono::steady_clock::now();
      }
    } else if (phase_ == 1) {
      publish(0.15, 0.5);
      if (phase_time >= 2.0) {
        const double delta = std::remainder(
          yaw(latest_.pose.pose.orientation) - yaw(straight_end_.pose.pose.orientation),
          2.0 * M_PI);
        if (std::abs(delta) < 0.45) {
          fail("Yaw change is below 0.45 rad");
          return;
        }
        phase_ = 2;
        phase_start_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "Motion checks passed; testing command timeout");
      }
    } else if (phase_ == 2 && phase_time >= 2.0) {
      if (std::abs(latest_.twist.twist.linear.x) > 0.03 ||
        std::abs(latest_.twist.twist.angular.z) > 0.05)
      {
        RCLCPP_ERROR(
          get_logger(), "Timeout velocities: linear=%.6f angular=%.6f",
          latest_.twist.twist.linear.x, latest_.twist.twist.angular.z);
        fail("Vehicle did not stop after cmd_vel_timeout");
        return;
      }
      for (int i = 0; i < 5; ++i) {publish(0.0, 0.0);}
      RCLCPP_INFO(get_logger(), "PASS: forward, turn, finite odometry, stop and timeout");
      result_ = 0;
      done_ = true;
      rclcpp::shutdown();
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry initial_;
  nav_msgs::msg::Odometry straight_end_;
  nav_msgs::msg::Odometry latest_;
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point phase_start_{std::chrono::steady_clock::now()};
  bool have_odom_{false};
  bool have_initial_{false};
  bool topic_checked_{false};
  bool done_{false};
  int phase_{0};
  int result_{1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<CarMotionSmokeNode>(options);
  rclcpp::spin(node);
  return node->result();
}
