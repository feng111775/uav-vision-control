#include "car_control/sim/final_lap_metrics.hpp"
#include "car_control/sim/d_task_track_geometry.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class CarFullLapAcceptanceNode : public rclcpp::Node
{
public:
  CarFullLapAcceptanceNode()
  : Node("car_full_lap_acceptance_node")
  {
    minimum_running_speed_ = declare_parameter("minimum_running_speed_mps", 0.05);
    maximum_linear_speed_ = declare_parameter("maximum_linear_speed_mps", 0.35);
    maximum_angular_speed_ = declare_parameter("maximum_angular_speed_radps", 3.0);
    wall_timeout_s_ = declare_parameter("wall_timeout_s", 120.0);
    initial_base_x_ = declare_parameter("initial_base_x", 1.50);
    initial_base_y_ = declare_parameter("initial_base_y", 1.80);
    initial_yaw_ = declare_parameter("initial_yaw", 1.5707963267948966);
    sensor_forward_offset_m_ = declare_parameter("sensor_forward_offset_m", 0.20);
    track_rms_limit_m_ = declare_parameter("track_rms_limit_m", 0.025);
    track_max_limit_m_ = declare_parameter("track_max_limit_m", 0.060);
    lost_line_limit_s_ = declare_parameter("lost_line_limit_s", 0.10);
    const auto csv_path = declare_parameter("csv_path", std::string(""));
    if (!csv_path.empty()) {
      csv_.open(csv_path);
      csv_ << "sim_time_s,stage,base_world_x,base_world_y,base_yaw,"
        "sensor_center_world_x,sensor_center_world_y,nearest_track_segment,"
        "sensor_center_cross_track_error_m,base_center_cross_track_error_m,"
        "line_error,detected,total_activation,commanded_linear_speed,"
        "commanded_angular_speed,marker,line_state\n";
    }
    auto qos = rclcpp::QoS(20);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        finite_ = finite_ && std::isfinite(p.x) && std::isfinite(p.y);
        odom_received_ = true;
      });
    ground_truth_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/car/ground_truth_pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        const auto & pose = msg->pose;
        const auto & p = pose.position;
        const auto & q = pose.orientation;
        const double yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        ground_truth_received_ =
          std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(yaw);
        if (ground_truth_received_) {
          base_world_x_ = p.x;
          base_world_y_ = p.y;
          base_world_yaw_ = yaw;
        }
      });
    values_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/car/line_sensor/values", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        sensor_received_ = !msg->data.empty();
        for (const float value : msg->data) {
          finite_ = finite_ && std::isfinite(value) && value >= 0.0F && value <= 1.0F;
        }
      });
    error_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/line_sensor/error", qos, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->data);
        line_error_ = msg->data;
      });
    detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/car/line_sensor/detected", qos, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        detected_ = msg->data;
      });
    activation_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/line_sensor/total_activation", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->data);
        total_activation_ = msg->data;
      });
    linear_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/control/commanded_linear_speed_mps", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->data);
        current_linear_ = msg->data;
        maximum_linear_seen_ = std::max(maximum_linear_seen_, std::abs(msg->data));
        speed_limited_ = speed_limited_ &&
          std::abs(msg->data) <= maximum_linear_speed_ + 1.0e-6;
        if (running_ && !finished_) {
          if (msg->data > 0.0) {
            minimum_positive_linear_ = std::min(minimum_positive_linear_, msg->data);
          }
          consecutive_low_speed_ =
            msg->data < minimum_running_speed_ ? consecutive_low_speed_ + 1 : 0;
          no_sustained_stop_ = no_sustained_stop_ && consecutive_low_speed_ < 3;
        }
        if (previous_line_state_ == "RECOVERY" && msg->data > 0.0) {
          minimum_recovery_speed_ = std::min(minimum_recovery_speed_, msg->data);
        }
        if (final_settle_started_) {
          final_metrics_.update_final_speed(msg->data);
        }
      });
    angular_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/control/commanded_angular_speed_radps", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->data);
        maximum_angular_seen_ = std::max(maximum_angular_seen_, std::abs(msg->data));
        speed_limited_ = speed_limited_ &&
          std::abs(msg->data) <= maximum_angular_speed_ + 1.0e-6;
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_controller/cmd_vel", qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        finite_ = finite_ && std::isfinite(msg->twist.linear.x) &&
          std::isfinite(msg->twist.angular.z);
        latest_sim_time_s_ =
          static_cast<double>(msg->header.stamp.sec) +
          static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
        cmd_received_ = true;
        sample_track(msg->twist.linear.x, msg->twist.angular.z);
      });
    line_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/control/line_state", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data == "FAULT") {
          faulted_ = true;
        }
        if (msg->data == "RECOVERY" && previous_line_state_ != "RECOVERY") {
          ++recovery_entries_;
          recovery_start_s_ = latest_sim_time_s_;
        } else if (msg->data != "RECOVERY" && previous_line_state_ == "RECOVERY") {
          recovery_duration_s_ += std::max(0.0, latest_sim_time_s_ - recovery_start_s_);
        }
        previous_line_state_ = msg->data;
      });
    stage_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/progress/stage", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        stage_ = msg->data;
        running_ = running_ || stage_ == "A_TO_B";
        if (final_settle_started_) {
          final_metrics_.update_stage(msg->data);
        }
      });
    marker_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/progress/last_marker", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data != "NONE" && msg->data != last_marker_) {
          markers_.push_back(msg->data);
          if (msg->data != "A" && !finished_ && current_linear_ < minimum_running_speed_) {
            marker_motion_valid_ = false;
          }
        }
        last_marker_ = msg->data;
        if (final_settle_started_) {
          final_metrics_.update_marker(msg->data);
        }
      });
    ab_time_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/progress/a_to_b_time_s", qos, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        ab_time_s_ = msg->data;
        if (final_settle_started_) {
          final_metrics_.update_ab_time(msg->data);
        }
      });
    lap_time_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/car/progress/lap_time_s", qos, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        lap_time_s_ = msg->data;
        if (final_settle_started_) {
          final_metrics_.update_lap_time(msg->data);
        }
      });
    ab_pass_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/car/progress/a_to_b_pass", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        ab_pass_ = msg->data;
        if (final_settle_started_) {
          final_metrics_.update_ab_pass(msg->data);
        }
      });
    lap_pass_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/car/progress/lap_pass", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        lap_pass_ = msg->data;
        if (final_settle_started_) {
          final_metrics_.update_lap_pass(msg->data);
        }
      });
    finished_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/car/progress/finished", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        finished_ = msg->data;
        if (msg->data && !final_settle_started_) {
          final_settle_started_ = true;
          final_settle_start_s_ = now().seconds();
          final_metrics_.begin();
        }
      });
    fault_sub_ = create_subscription<std_msgs::msg::String>(
      "/car/control/fault_reason", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (!msg->data.empty()) {
          faulted_ = true;
          fault_reason_ = msg->data;
        }
        if (final_settle_started_) {
          final_metrics_.update_fault_reason(msg->data);
        }
      });
    start_wall_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {check();});
  }

  bool success() const noexcept {return success_;}

private:
  std::string marker_sequence() const
  {
    std::ostringstream stream;
    for (std::size_t i = 0; i < markers_.size(); ++i) {
      if (i != 0U) {
        stream << ',';
      }
      stream << markers_[i];
    }
    return stream.str();
  }

  static car_control::sim::TrackSegment stage_segment(const std::string & stage)
  {
    using car_control::sim::TrackSegment;
    if (stage == "A_TO_B") {return TrackSegment::A_TO_B;}
    if (stage == "B_TO_C") {return TrackSegment::B_TO_C;}
    if (stage == "C_TO_D") {return TrackSegment::C_TO_D;}
    if (stage == "D_TO_A") {return TrackSegment::D_TO_A;}
    return TrackSegment::INVALID;
  }

  void sample_track(double linear, double angular)
  {
    using car_control::sim::Point2;
    using car_control::sim::TrackSegment;
    const auto segment = stage_segment(stage_);
    if (segment == TrackSegment::INVALID || !ground_truth_received_ || !sensor_received_) {
      return;
    }
    const double sensor_x =
      base_world_x_ + std::cos(base_world_yaw_) * sensor_forward_offset_m_;
    const double sensor_y =
      base_world_y_ + std::sin(base_world_yaw_) * sensor_forward_offset_m_;
    const double sensor_error = track_.distance_to_segment({sensor_x, sensor_y}, segment);
    const double base_error =
      track_.distance_to_segment({base_world_x_, base_world_y_}, segment);
    const auto nearest = track_.nearest({sensor_x, sensor_y});
    auto & stats = track_stats_[static_cast<std::size_t>(segment)];
    ++stats.samples;
    stats.sensor_sum_sq += sensor_error * sensor_error;
    stats.sensor_max = std::max(stats.sensor_max, sensor_error);
    stats.base_sum_sq += base_error * base_error;
    stats.base_max = std::max(stats.base_max, base_error);
    const double dt = last_track_sample_s_ > 0.0 ?
      std::max(0.0, latest_sim_time_s_ - last_track_sample_s_) : 0.0;
    stats.lost_current_s = detected_ ? 0.0 : stats.lost_current_s + dt;
    stats.lost_max_s = std::max(stats.lost_max_s, stats.lost_current_s);
    if (!detected_) {
      ++stats.lost_samples;
    }
    stats.max_line_error = std::max(stats.max_line_error, std::abs(line_error_));
    stats.max_angular = std::max(stats.max_angular, std::abs(angular));
    last_track_sample_s_ = latest_sim_time_s_;
    if (csv_) {
      csv_ << latest_sim_time_s_ << ',' << stage_ << ',' <<
        base_world_x_ << ',' << base_world_y_ << ',' << base_world_yaw_ << ',' <<
        sensor_x << ',' << sensor_y << ',' <<
        car_control::sim::DTaskTrackGeometry::segment_name(nearest.segment) << ',' <<
        sensor_error << ',' << base_error << ',' << line_error_ << ',' <<
        (detected_ ? 1 : 0) << ',' << total_activation_ << ',' <<
        linear << ',' << angular << ',' << last_marker_ << ',' <<
        previous_line_state_ << '\n';
    }
  }

  bool track_metrics_pass() const
  {
    for (const auto & stats : track_stats_) {
      if (stats.samples == 0) {
        return false;
      }
      const double rms = std::sqrt(stats.sensor_sum_sq / stats.samples);
      if (!std::isfinite(rms) || rms > track_rms_limit_m_ ||
        stats.sensor_max > track_max_limit_m_ ||
        stats.lost_max_s > lost_line_limit_s_)
      {
        return false;
      }
    }
    return true;
  }

  void log_track_metrics() const
  {
    for (std::size_t index = 0; index < track_stats_.size(); ++index) {
      const auto & stats = track_stats_[index];
      const auto segment = static_cast<car_control::sim::TrackSegment>(index);
      const double sensor_rms = stats.samples > 0 ?
        std::sqrt(stats.sensor_sum_sq / stats.samples) : NAN;
      const double base_rms = stats.samples > 0 ?
        std::sqrt(stats.base_sum_sq / stats.samples) : NAN;
      RCLCPP_INFO(
        get_logger(),
        "TRACK_METRIC segment=%s samples=%zu sensor_rms=%.6f sensor_max=%.6f "
        "base_rms=%.6f base_max=%.6f lost_samples=%zu lost_max=%.6f "
        "max_line_error=%.6f max_angular=%.6f pass=%s",
        car_control::sim::DTaskTrackGeometry::segment_name(segment),
        stats.samples, sensor_rms, stats.sensor_max, base_rms, stats.base_max,
        stats.lost_samples, stats.lost_max_s, stats.max_line_error,
        stats.max_angular,
        stats.samples > 0 && sensor_rms <= track_rms_limit_m_ &&
        stats.sensor_max <= track_max_limit_m_ &&
        stats.lost_max_s <= lost_line_limit_s_ ? "true" : "false");
    }
  }

  void check()
  {
    if (faulted_ || !finite_ || !speed_limited_ || !no_sustained_stop_) {
      finish(false, faulted_ ? "FAULT: " + fault_reason_ :
        (!finite_ ? "non-finite value" :
        (!speed_limited_ ? "speed limit exceeded" : "sustained stop before FINISHED")));
      return;
    }
    if (final_settle_started_) {
      final_metrics_.update_recovery(recovery_entries_, recovery_duration_s_);
      const auto final_result = final_metrics_.evaluate();
      const double settle_elapsed_s = now().seconds() - final_settle_start_s_;
      const std::vector<std::string> expected{"B", "C", "D", "A"};
      if (final_result.ready && settle_elapsed_s >= 0.06) {
        const bool passed = final_result.passed && markers_ == expected &&
          running_ && marker_motion_valid_ && odom_received_ &&
          ground_truth_received_ && sensor_received_ && cmd_received_ &&
          track_metrics_pass();
        finish(
          passed, passed ? "full lap accepted" :
          "FINAL_METRIC_INVALID: " + final_result.diagnostic);
        return;
      }
      if (std::isfinite(settle_elapsed_s) && settle_elapsed_s >= 1.0) {
        finish(false, "FINAL_METRIC_TIMEOUT: " + final_result.diagnostic);
        return;
      }
    }
    if (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_wall_).count() > wall_timeout_s_)
    {
      finish(false, "wall-clock timeout before FINISHED");
    }
  }

  void finish(bool passed, const std::string & reason)
  {
    if (previous_line_state_ == "RECOVERY") {
      recovery_duration_s_ += std::max(0.0, latest_sim_time_s_ - recovery_start_s_);
    }
    success_ = passed;
    log_track_metrics();
    RCLCPP_INFO(
      get_logger(),
      "%s: markers=%s A-B=%.3fs lap=%.3fs min_v=%.3f max_v=%.3f max_w=%.3f "
      "recovery_entries=%d recovery_time=%.3fs fault=%s final_v=%.3f "
      "recovery_min_v=%.3f final_stage=%s finished=%s last_marker=%s "
      "a_to_b_pass=%s lap_pass=%s reason=%s",
      passed ? "PASS" : "FAIL", marker_sequence().c_str(), ab_time_s_, lap_time_s_,
      std::isfinite(minimum_positive_linear_) ? minimum_positive_linear_ : 0.0,
      maximum_linear_seen_, maximum_angular_seen_, recovery_entries_, recovery_duration_s_,
      faulted_ ? "yes" : "no", current_linear_,
      std::isfinite(minimum_recovery_speed_) ? minimum_recovery_speed_ : 0.0,
      stage_.c_str(),
      finished_ ? "true" : "false", last_marker_.c_str(),
      ab_pass_ ? "true" : "false", lap_pass_ ? "true" : "false", reason.c_str());
    rclcpp::shutdown();
  }

  double minimum_running_speed_{0.05};
  double maximum_linear_speed_{0.35};
  double maximum_angular_speed_{3.0};
  double wall_timeout_s_{120.0};
  double initial_base_x_{1.50};
  double initial_base_y_{1.80};
  double initial_yaw_{1.5707963267948966};
  double sensor_forward_offset_m_{0.20};
  double track_rms_limit_m_{0.025};
  double track_max_limit_m_{0.060};
  double lost_line_limit_s_{0.10};
  bool success_{false};
  bool finite_{true};
  bool faulted_{false};
  bool running_{false};
  bool finished_{false};
  bool ab_pass_{false};
  bool lap_pass_{false};
  bool odom_received_{false};
  bool ground_truth_received_{false};
  bool sensor_received_{false};
  bool cmd_received_{false};
  bool speed_limited_{true};
  bool no_sustained_stop_{true};
  bool marker_motion_valid_{true};
  bool final_settle_started_{false};
  int consecutive_low_speed_{0};
  int recovery_entries_{0};
  double current_linear_{0.0};
  double minimum_positive_linear_{std::numeric_limits<double>::infinity()};
  double maximum_linear_seen_{0.0};
  double maximum_angular_seen_{0.0};
  double ab_time_s_{0.0};
  double lap_time_s_{0.0};
  double latest_sim_time_s_{0.0};
  double recovery_start_s_{0.0};
  double recovery_duration_s_{0.0};
  double minimum_recovery_speed_{std::numeric_limits<double>::infinity()};
  double final_settle_start_s_{0.0};
  double base_world_x_{0.0};
  double base_world_y_{0.0};
  double base_world_yaw_{0.0};
  double line_error_{0.0};
  double total_activation_{0.0};
  double last_track_sample_s_{0.0};
  bool detected_{false};
  std::string stage_{"WAIT_START"};
  std::string last_marker_{"NONE"};
  std::string previous_line_state_;
  std::string fault_reason_;
  car_control::sim::FinalLapMetrics final_metrics_;
  struct TrackStats
  {
    std::size_t samples{0};
    std::size_t lost_samples{0};
    double sensor_sum_sq{0.0};
    double sensor_max{0.0};
    double base_sum_sq{0.0};
    double base_max{0.0};
    double lost_current_s{0.0};
    double lost_max_s{0.0};
    double max_line_error{0.0};
    double max_angular{0.0};
  };
  car_control::sim::DTaskTrackGeometry track_;
  std::array<TrackStats, 4> track_stats_;
  std::ofstream csv_;
  std::vector<std::string> markers_;
  std::chrono::steady_clock::time_point start_wall_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr values_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr activation_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angular_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr line_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ab_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lap_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ab_pass_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lap_pass_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarFullLapAcceptanceNode>();
  rclcpp::spin(node);
  return node->success() ? 0 : 1;
}
