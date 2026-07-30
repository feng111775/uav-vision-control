#include "car_control/core/line_pd_controller.hpp"
#include "car_control/core/line_recovery.hpp"
#include "car_control/core/progress_state_machine.hpp"
#include "car_control/core/speed_planner.hpp"
#include "car_control/sim/control_safety.hpp"
#include "car_control/sim/progress_detector.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>

namespace car_control::sim
{
namespace
{
std::string line_state_name(core::LineState state)
{
  switch (state) {
    case core::LineState::TRACKING: return "TRACKING";
    case core::LineState::RECOVERY: return "RECOVERY";
    default: return "FAULT";
  }
}

std::string progress_name(core::ProgressState state)
{
  switch (state) {
    case core::ProgressState::WAIT_START: return "WAIT_START";
    case core::ProgressState::A_TO_B: return "A_TO_B";
    case core::ProgressState::B_TO_C: return "B_TO_C";
    case core::ProgressState::C_TO_D: return "C_TO_D";
    case core::ProgressState::D_TO_A: return "D_TO_A";
    default: return "FINISHED";
  }
}

std::string marker_name(core::Marker marker)
{
  switch (marker) {
    case core::Marker::A: return "A";
    case core::Marker::B: return "B";
    case core::Marker::C: return "C";
    case core::Marker::D: return "D";
    default: return "NONE";
  }
}
}  // namespace

class LineFollowControllerNode : public rclcpp::Node
{
public:
  LineFollowControllerNode()
  : Node("line_follow_controller_node"),
    pd_({
      declare_parameter("line_pd.kp", 0.08),
      declare_parameter("line_pd.kd", 0.004),
      declare_parameter("line_pd.output_limit", 0.12),
      declare_parameter("line_pd.derivative_filter", 0.7)}),
    planner_({
      declare_parameter("maximum_speed_mps", 0.35),
      declare_parameter("minimum_tracking_speed_mps", 0.12),
      declare_parameter("speed_planner.error_slowdown_gain", 0.08),
      declare_parameter("speed_planner.maximum_wheel_speed_mps", 0.45),
      declare_parameter("recovery_speed_mps", 0.10)}),
    recovery_({
      declare_parameter("recovery.timeout_s", 1.0),
      declare_parameter("recovery.steering_command", 0.07)}),
    progress_({
      declare_parameter("progress.minimum_distance_m", 0.5),
      declare_parameter("progress.lockout_time_s", 0.5),
      declare_parameter("progress.a_to_b_limit_s", 15.0),
      declare_parameter("progress.lap_limit_s", 90.0)}),
    detector_(make_detector())
  {
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/car/ground_truth_odom");
    auto_start_ = declare_parameter("auto_start", false);
    stop_after_finish_ = declare_parameter("stop_after_finish", true);
    control_rate_hz_ = declare_parameter("control_rate_hz", 50.0);
    sensor_timeout_s_ = declare_parameter("sensor_timeout_s", 0.20);
    odom_timeout_s_ = declare_parameter("odom_timeout_s", 0.30);
    startup_fresh_cycles_ = declare_parameter("startup_fresh_cycles", 3);
    startup_gate_ = StartupFreshnessGate(startup_fresh_cycles_);
    base_speed_mps_ = declare_parameter("base_speed_mps", 0.25);
    wheel_separation_m_ = declare_parameter("wheel_separation_m", 0.32);
    if (!detector_.valid() || !startup_gate_.valid() ||
      !std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("invalid line follow configuration");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_controller/cmd_vel", 10);
    line_state_pub_ = create_publisher<std_msgs::msg::String>("/car/control/line_state", 10);
    fault_pub_ = create_publisher<std_msgs::msg::String>("/car/control/fault_reason", 10);
    linear_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/car/control/commanded_linear_speed_mps", 10);
    angular_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/car/control/commanded_angular_speed_radps", 10);
    stage_pub_ = create_publisher<std_msgs::msg::String>("/car/progress/stage", 10);
    marker_pub_ = create_publisher<std_msgs::msg::String>("/car/progress/last_marker", 10);
    ab_time_pub_ = create_publisher<std_msgs::msg::Float64>("/car/progress/a_to_b_time_s", 10);
    lap_time_pub_ = create_publisher<std_msgs::msg::Float64>("/car/progress/lap_time_s", 10);
    ab_pass_pub_ = create_publisher<std_msgs::msg::Bool>("/car/progress/a_to_b_pass", 10);
    lap_pass_pub_ = create_publisher<std_msgs::msg::Bool>("/car/progress/lap_pass", 10);
    finished_pub_ = create_publisher<std_msgs::msg::Bool>("/car/progress/finished", 10);

    values_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/car/line_sensor/values", 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        values_valid_ = !msg->data.empty();
        for (const float value : msg->data) {
          values_valid_ = values_valid_ && std::isfinite(value) && value >= 0.0F && value <= 1.0F;
        }
        sensor_snapshots_.update(
          SensorSnapshotSequence::Input::VALUES, values_valid_, now().seconds());
      });
    error_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/line_sensor/error", 10, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        line_error_ = msg->data;
        error_valid_ = std::isfinite(line_error_);
        sensor_snapshots_.update(
          SensorSnapshotSequence::Input::ERROR, error_valid_, now().seconds());
      });
    detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/car/line_sensor/detected", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        detected_ = msg->data;
        sensor_snapshots_.update(
          SensorSnapshotSequence::Input::DETECTED, true, now().seconds());
      });
    activation_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/line_sensor/total_activation", 10,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        activation_valid_ = std::isfinite(msg->data);
        sensor_snapshots_.update(
          SensorSnapshotSequence::Input::ACTIVATION, activation_valid_, now().seconds());
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        if (!std::isfinite(x) || !std::isfinite(y)) {
          odom_valid_ = false;
          return;
        }
        if (odom_received_) {
          cumulative_distance_m_ += std::hypot(x - odom_x_, y - odom_y_);
        }
        odom_x_ = x;
        odom_y_ = y;
        odom_valid_ = true;
        odom_received_ = true;
        odom_stamp_ = now();
        ++odom_receive_sequence_;
      });
    ground_truth_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/car/ground_truth_pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto & position = msg->pose.position;
        ground_truth_valid_ = std::isfinite(position.x) && std::isfinite(position.y);
        if (ground_truth_valid_) {
          ground_truth_x_ = position.x;
          ground_truth_y_ = position.y;
          ground_truth_received_ = true;
        }
      });
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/car/start",
      [this](
        std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (started_) {
          response->success = false;
          response->message = "already started; state was not reset";
          return;
        }
        const rclcpp::Time stamp = now();
        const InputFreshness freshness = input_freshness(stamp);
        if (!freshness.fresh) {
          response->success = false;
          response->message = "start rejected: " + freshness.reason;
          return;
        }
        if (startup_gate_.consecutive_cycles() < startup_fresh_cycles_) {
          response->success = false;
          response->message = "start rejected: waiting for distinct fresh input frames";
          return;
        }
        start_locked(stamp, freshness);
        response->success = true;
        response->message = "line following started";
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_), [this]() {control_tick();});
  }

private:
  ProgressDetector make_detector()
  {
    ProgressDetectorConfig config;
    config.initial_base_x = declare_parameter("initial_base_x", 1.50);
    config.initial_base_y = declare_parameter("initial_base_y", 1.80);
    config.initial_yaw = declare_parameter("initial_yaw", 1.5707963267948966);
    config.node_radius_m = declare_parameter("progress.node_radius_m", 0.25);
    config.a = {declare_parameter("progress.a_x", 1.50), declare_parameter("progress.a_y", 2.00)};
    config.b = {declare_parameter("progress.b_x", 1.50), declare_parameter("progress.b_y", 3.50)};
    config.c = {declare_parameter("progress.c_x", 3.00), declare_parameter("progress.c_y", 3.50)};
    config.d = {declare_parameter("progress.d_x", 3.00), declare_parameter("progress.d_y", 2.00)};
    return ProgressDetector(config);
  }

  bool inputs_ready() const
  {
    return sensor_snapshots_.sequence() > 0 && sensor_snapshots_.inputs_valid() &&
      values_valid_ && error_valid_ && activation_valid_ &&
      odom_received_ && odom_valid_ && ground_truth_received_ &&
      ground_truth_valid_ && std::isfinite(line_error_);
  }

  double complete_sensor_stamp_s() const
  {
    return sensor_snapshots_.receive_time_s();
  }

  InputFreshness input_freshness(const rclcpp::Time & stamp) const
  {
    return evaluate_input_freshness(
      stamp.seconds(), complete_sensor_stamp_s(), odom_stamp_.seconds(),
      sensor_timeout_s_, odom_timeout_s_, inputs_ready());
  }

  void start_locked(const rclcpp::Time & stamp, const InputFreshness & freshness)
  {
    pd_.reset();
    recovery_.reset();
    progress_.reset();
    detector_.reset();
    cumulative_distance_m_ = 0.0;
    last_marker_ = core::Marker::NONE;
    start_time_ = stamp;
    last_control_time_ = stamp;
    progress_output_ = progress_.update(true, core::Marker::NONE, 0.0, 0.0);
    started_ = true;
    const double wait_s = startup_wait_start_s_ > 0.0 ?
      stamp.seconds() - startup_wait_start_s_ : 0.0;
    const auto first_odom_sequence = startup_gate_.first_odom_sequence();
    const auto first_sensor_sequence = startup_gate_.first_sensor_sequence();
    const auto last_odom_sequence = startup_gate_.last_odom_sequence();
    const auto last_sensor_sequence = startup_gate_.last_sensor_sequence();
    RCLCPP_INFO(
      get_logger(),
      "line following started: auto=%s wait=%.3fs "
      "odom_sequence=%llu..%llu sensor_snapshot_sequence=%llu..%llu "
      "odom_age=%.6fs sensor_age=%.6fs",
      auto_start_ ? "true" : "false", wait_s,
      static_cast<unsigned long long>(first_odom_sequence),
      static_cast<unsigned long long>(last_odom_sequence),
      static_cast<unsigned long long>(first_sensor_sequence),
      static_cast<unsigned long long>(last_sensor_sequence),
      freshness.odom_age_s, freshness.sensor_age_s);
    startup_gate_.reset();
  }

  void fault_locked(const std::string & reason, const rclcpp::Time & stamp)
  {
    faulted_ = true;
    fault_reason_ = reason;
    publish_locked(0.0, 0.0, "FAULT", stamp);
  }

  void control_tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time stamp = now();
    if (!started_) {
      const InputFreshness freshness = input_freshness(stamp);
      if (stamp.seconds() > 0.0 && startup_wait_start_s_ <= 0.0) {
        startup_wait_start_s_ = stamp.seconds();
      }
      const bool gate_ready = startup_gate_.update(
        freshness.fresh, odom_receive_sequence_, sensor_snapshots_.sequence());
      if (auto_start_ && gate_ready) {
        start_locked(stamp, freshness);
        return;
      } else {
        publish_locked(0.0, 0.0, "WAITING", stamp);
        return;
      }
    }
    if (faulted_) {
      publish_locked(0.0, 0.0, "FAULT", stamp);
      return;
    }
    const double now_s = stamp.seconds();
    const double dt = (stamp - last_control_time_).seconds();
    if (dt == 0.0) {
      return;
    }
    last_control_time_ = stamp;
    if (!std::isfinite(dt) || dt < 0.0 || !inputs_ready()) {
      fault_locked("invalid dt or non-finite input", stamp);
      return;
    }
    const std::string timing_fault = input_fault_reason(
      now_s, complete_sensor_stamp_s(), odom_stamp_.seconds(),
      sensor_timeout_s_, odom_timeout_s_);
    if (!timing_fault.empty()) {
      RCLCPP_ERROR(
        get_logger(), "%s: now=%.6f sensor_stamp=%.6f sensor_age=%.6f "
        "odom_stamp=%.6f odom_age=%.6f sensor_sequence=%llu odom_sequence=%llu",
        timing_fault.c_str(), now_s, complete_sensor_stamp_s(),
        now_s - complete_sensor_stamp_s(), odom_stamp_.seconds(),
        now_s - odom_stamp_.seconds(),
        static_cast<unsigned long long>(sensor_snapshots_.sequence()),
        static_cast<unsigned long long>(odom_receive_sequence_));
      fault_locked(timing_fault, stamp);
      return;
    }
    const auto recovery = recovery_.update(detected_, line_error_, dt);
    if (!recovery.valid) {
      fault_locked("line recovery rejected input", stamp);
      return;
    }
    double steering = recovery.suggested_steering;
    if (recovery.state == core::LineState::TRACKING) {
      const auto correction = pd_.update(line_error_, dt);
      if (!correction.valid) {
        fault_locked("line PD rejected input", stamp);
        return;
      }
      steering = correction.value;
    }
    const auto wheels = core::plan_wheel_speeds(
      planner_, base_speed_mps_, line_error_, steering, recovery.state);
    const auto command = wheel_targets_to_chassis(
      wheels.left_m_s, wheels.right_m_s, wheel_separation_m_);
    if (!wheels.valid || !command.valid) {
      fault_locked("speed planner or chassis conversion rejected input", stamp);
      return;
    }

    const double competition_time = (stamp - start_time_).seconds();
    const core::Marker marker = detector_.update_world(
      ground_truth_x_, ground_truth_y_, progress_.state());
    progress_output_ = progress_.update(
      false, marker, cumulative_distance_m_, competition_time);
    if (!progress_output_.valid) {
      fault_locked("progress state machine rejected input", stamp);
      return;
    }
    if (progress_output_.marker_accepted) {
      last_marker_ = marker;
    }
    if (progress_output_.state == core::ProgressState::FINISHED && stop_after_finish_) {
      publish_locked(0.0, 0.0, "FINISHED", stamp);
      return;
    }
    publish_locked(command.linear_x, command.angular_z, line_state_name(recovery.state), stamp);
  }

  void publish_locked(
    double linear, double angular, const std::string & line_state,
    const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;
    cmd_pub_->publish(cmd);
    std_msgs::msg::String text;
    text.data = line_state;
    line_state_pub_->publish(text);
    text.data = fault_reason_;
    fault_pub_->publish(text);
    std_msgs::msg::Float64 number;
    number.data = linear;
    linear_pub_->publish(number);
    number.data = angular;
    angular_pub_->publish(number);
    text.data = progress_name(progress_.state());
    stage_pub_->publish(text);
    text.data = marker_name(last_marker_);
    marker_pub_->publish(text);
    number.data = progress_output_.ab_elapsed_s;
    ab_time_pub_->publish(number);
    number.data = progress_output_.lap_elapsed_s;
    lap_time_pub_->publish(number);
    std_msgs::msg::Bool flag;
    flag.data = progress_output_.ab_within_target;
    ab_pass_pub_->publish(flag);
    flag.data = progress_output_.lap_within_target;
    lap_pass_pub_->publish(flag);
    flag.data = progress_.state() == core::ProgressState::FINISHED;
    finished_pub_->publish(flag);
  }

  core::LinePDController pd_;
  core::SpeedPlannerConfig planner_;
  core::LineRecovery recovery_;
  core::ProgressStateMachine progress_;
  ProgressDetector detector_;
  StartupFreshnessGate startup_gate_{3};
  SensorSnapshotSequence sensor_snapshots_;
  bool auto_start_{false};
  bool stop_after_finish_{true};
  double control_rate_hz_{50.0};
  double sensor_timeout_s_{0.2};
  double odom_timeout_s_{0.3};
  int startup_fresh_cycles_{3};
  double base_speed_mps_{0.25};
  double wheel_separation_m_{0.32};
  std::mutex mutex_;
  bool values_valid_{false};
  bool error_valid_{false};
  bool activation_valid_{false};
  bool odom_received_{false};
  bool odom_valid_{false};
  bool ground_truth_received_{false};
  bool ground_truth_valid_{false};
  bool detected_{false};
  bool started_{false};
  bool faulted_{false};
  double line_error_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double ground_truth_x_{0.0};
  double ground_truth_y_{0.0};
  double cumulative_distance_m_{0.0};
  double startup_wait_start_s_{0.0};
  std::uint64_t odom_receive_sequence_{0};
  std::string fault_reason_;
  core::Marker last_marker_{core::Marker::NONE};
  core::ProgressOutput progress_output_;
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr values_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr activation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr line_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr linear_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ab_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lap_time_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ab_pass_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lap_pass_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace car_control::sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<car_control::sim::LineFollowControllerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("line_follow_controller_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
