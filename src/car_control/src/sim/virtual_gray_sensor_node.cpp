#include "car_control/sim/virtual_gray_sensor.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

namespace car_control::sim
{

class VirtualGraySensorNode : public rclcpp::Node
{
public:
  VirtualGraySensorNode()
  : Node("virtual_gray_sensor_node")
  {
    const std::string odom_topic = declare_parameter("odom_topic", "/diff_drive_controller/odom");
    const std::string values_topic = declare_parameter("values_topic", "/car/line_sensor/values");
    const std::string error_topic = declare_parameter("error_topic", "/car/line_sensor/error");
    const std::string detected_topic = declare_parameter("detected_topic", "/car/line_sensor/detected");
    const std::string activation_topic =
      declare_parameter("total_activation_topic", "/car/line_sensor/total_activation");
    const double rate = declare_parameter("publish_rate_hz", 50.0);
    initial_x_ = declare_parameter("initial_world_x", 1.50);
    initial_y_ = declare_parameter("initial_world_y", 1.80);
    initial_yaw_ = declare_parameter("initial_world_yaw", 1.5707963267948966);

    VirtualGraySensorConfig config;
    config.sensor_forward_offset_m = declare_parameter("sensor_forward_offset_m", 0.20);
    config.channel_positions_m = declare_parameter<std::vector<double>>(
      "channel_positions_m", {-0.045, -0.030, -0.015, 0.0, 0.015, 0.030, 0.045});
    config.line_width_m = declare_parameter("line_width_m", 0.020);
    config.sample_width_m = declare_parameter("sample_width_m", 0.008);
    config.black_line_is_active = declare_parameter("black_line_is_active", true);
    config.minimum_activation = declare_parameter("minimum_activation", 0.2);
    config.brightness_gain = declare_parameter("brightness_gain", 1.0);
    config.brightness_bias = declare_parameter("brightness_bias", 0.0);
    config.noise_stddev = declare_parameter("noise_stddev", 0.0);
    const auto seed = declare_parameter("random_seed", 2026);
    config.random_seed = static_cast<std::uint32_t>(std::max(seed, 0L));
    sensor_ = std::make_unique<VirtualGraySensor>(config);
    if (!sensor_->valid() || !std::isfinite(rate) || rate <= 0.0 ||
      !std::isfinite(initial_x_) || !std::isfinite(initial_y_) || !std::isfinite(initial_yaw_))
    {
      throw std::invalid_argument("invalid virtual gray sensor configuration");
    }

    values_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(values_topic, 10);
    error_pub_ = create_publisher<std_msgs::msg::Float64>(error_topic, 10);
    detected_pub_ = create_publisher<std_msgs::msg::Bool>(detected_topic, 10);
    activation_pub_ = create_publisher<std_msgs::msg::Float64>(activation_topic, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_ = std::move(msg);
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {publish_sample();});
  }

private:
  void publish_sample()
  {
    nav_msgs::msg::Odometry::ConstSharedPtr odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom = odom_;
    }
    if (!odom) {
      return;
    }
    const auto & p = odom->pose.pose.position;
    const auto & q = odom->pose.pose.orientation;
    const double odom_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double c = std::cos(initial_yaw_);
    const double s = std::sin(initial_yaw_);
    const auto result = sensor_->sample(
      initial_x_ + c * p.x - s * p.y,
      initial_y_ + s * p.x + c * p.y,
      initial_yaw_ + odom_yaw);
    if (!result.valid) {
      return;
    }
    std_msgs::msg::Float32MultiArray values;
    values.data.reserve(result.values.size());
    for (const double value : result.values) {
      values.data.push_back(static_cast<float>(value));
    }
    std_msgs::msg::Float64 error;
    error.data = result.line.error;
    std_msgs::msg::Bool detected;
    detected.data = result.line.detected;
    std_msgs::msg::Float64 activation;
    activation.data = result.line.total_activation;
    values_pub_->publish(values);
    error_pub_->publish(error);
    detected_pub_->publish(detected);
    activation_pub_->publish(activation);
  }

  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  std::unique_ptr<VirtualGraySensor> sensor_;
  std::mutex mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr values_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr activation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace car_control::sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<car_control::sim::VirtualGraySensorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("virtual_gray_sensor_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
