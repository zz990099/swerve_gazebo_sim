// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "swerve_chassis_core/setpoint_generator.hpp"
#include "swerve_chassis_msgs/msg/chassis_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
namespace core = swerve_chassis_core;
using Steady = std::chrono::steady_clock;
class Controller : public rclcpp::Node
{
 public:
  Controller() : Node("swerve_controller")
  {
    g_.wheelbase = positive("wheelbase", 0.6);
    g_.track_width = positive("track_width", 0.5);
    g_.radius = positive("wheel_radius", 0.1);
    l_.steering = positive("steering_limit", core::pi / 2);
    l_.steering_rate = positive("max_steering_rate", 2.5);
    l_.wheel_speed = positive("max_wheel_speed", 20);
    l_.wheel_acceleration = positive("max_wheel_acceleration", 40);
    l_.linear_speed = positive("max_linear_speed", 0.8);
    l_.angular_speed = positive("max_angular_speed", 0.8);
    l_.linear_acceleration = positive("max_linear_acceleration", 0.9);
    l_.angular_acceleration = positive("max_angular_acceleration", 1.3);
    l_.tracking_tolerance = positive("steering_tracking_tolerance", 0.05);
    l_.stopped_speed = positive("stopped_wheel_speed", 0.05);
    generator_ = std::make_unique<core::Generator>(g_, l_);
    cmd_timeout_ = positive("cmd_timeout", 0.5);
    wall_timeout_ = positive("wall_timeout", 0.5);
    feedback_timeout_ = positive("feedback_timeout", 0.25);
    double hz = positive("update_rate", 100);
    if (hz != 100) {
      throw std::invalid_argument("Requires 100 Hz control");
    }
    pose_variance_ = positive("pose_variance", 0.02);
    twist_variance_ = positive("twist_variance", 0.02);
    const auto prefix = declare_parameter<std::string>("joint_prefix", "");
    const auto frames = declare_parameter<std::string>("frame_prefix", "");
    body_frame_ = frames + "base_footprint";
    odom_frame_ = declare_parameter<std::string>("odom_frame", "");
    child_frame_ = declare_parameter<std::string>("odom_child_frame", "");
    if (odom_frame_.empty()) {
      odom_frame_ = frames + "odom";
    }
    if (child_frame_.empty()) {
      child_frame_ = body_frame_;
    }
    if (odom_frame_ == child_frame_) {
      throw std::invalid_argument("Odometry frames must differ");
    }
    for (const auto & c : {"fl", "fr", "rl", "rr"}) {
      steering_names_.push_back(prefix + c + "_steering_joint");
      wheel_names_.push_back(prefix + c + "_wheel_joint");
    }
    auto odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    if (declare_parameter<bool>("publish_odom", true)) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    }
    if (declare_parameter<bool>("publish_odom_tf", true)) {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("wheel_controller/commands", 1);
    steering_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("steering_controller/commands", 1);
    state_pub_ = create_publisher<swerve_chassis_msgs::msg::ChassisState>(
        "chassis_controller/state", rclcpp::SensorDataQoS());
    feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { feedback(*msg); });
    // Exactly one source. Legacy Nav2 Twist must be explicitly selected.
    const auto input = declare_parameter<std::string>("command_type", "stamped");
    if (input == "stamped") {
      command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          "cmd_vel", 1, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
            const double t = stamp(msg->header.stamp);
            observe_clock();
            if (msg->header.frame_id != body_frame_ || !valid_stamp(msg->header.stamp) ||
                t > now().seconds() + 0.01 || now().seconds() - t > cmd_timeout_ ||
                (have_command_ && t <= command_stamp_)) {
              stop(1);
              return;
            }
            command(msg->twist, t);
          });
    } else if (input == "unstamped") {
      legacy_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            observe_clock();
            command(*msg, now().seconds());
          });
    } else {
      throw std::invalid_argument("command_type must be stamped or unstamped");
    }
    // Parameters are startup-only, including geometry shared with URDF and
    // MPPI.
    parameter_callback_ =
        add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "Restart controller to change configuration";
          return result;
        });
    last_clock_ = now().seconds();
    last_tick_ = last_clock_;
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });
  }
  void shutdown_stop()
  {
    stop(1);
    publish_targets();
  }

 private:
  double positive(const std::string & name, double value)
  {
    double v = declare_parameter<double>(name, value);
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return v;
  }
  static double stamp(const builtin_interfaces::msg::Time & t)
  {
    return t.sec + t.nanosec * 1e-9;
  }
  static bool valid_stamp(const builtin_interfaces::msg::Time & t)
  {
    return t.sec >= 0 && t.nanosec < 1000000000;
  }
  static geometry_msgs::msg::Twist message(core::Twist t)
  {
    geometry_msgs::msg::Twist out;
    out.linear.x = t.vx;
    out.linear.y = t.vy;
    out.angular.z = t.wz;
    return out;
  }
  static double age(Steady::time_point t)
  {
    return std::chrono::duration<double>(Steady::now() - t).count();
  }
  void stop(uint8_t fault)
  {
    fault_ = fault;
    state_.command.speed.fill(0);
    if (have_feedback_) {
      for (size_t i = 0; i < 4; ++i) {
        state_.command.steering[i] = std::clamp(measured_.steering[i], -l_.steering, l_.steering);
      }
    }
    state_.valid = false;
    execution_ = core::Reason::stopped;
  }
  void observe_clock()
  {
    double t = now().seconds();
    if (t < last_clock_) {
      stop(3);
      have_command_ = have_feedback_ = false;
      have_odom_ = false;
      last_tick_ = t;  // Pose is preserved: a clock reset does not teleport the robot.
    }
    last_clock_ = t;
  }
  bool feedback_fresh() const
  {
    return have_feedback_ && now().seconds() >= feedback_stamp_ - 0.01 &&
           now().seconds() - feedback_stamp_ <= feedback_timeout_ &&
           age(feedback_wall_) <= feedback_timeout_;
  }
  bool command_fresh() const
  {
    return have_command_ && now().seconds() >= command_stamp_ - 0.01 &&
           now().seconds() - command_stamp_ <= cmd_timeout_ && age(command_wall_) <= wall_timeout_;
  }
  void command(const geometry_msgs::msg::Twist & msg, double t)
  {
    core::Twist value{msg.linear.x, msg.linear.y, msg.angular.z};
    if (!core::finite(value) || msg.linear.z != 0 || msg.angular.x != 0 || msg.angular.y != 0) {
      stop(1);
      return;
    }
    // Fault recovery requires a fresh zero command plus healthy stopped
    // encoders.
    if (fault_) {
      bool stopped = feedback_fresh();
      for (double speed : measured_.speed) {
        stopped &= std::abs(speed) <= l_.stopped_speed;
      }
      if (!stopped || std::hypot(value.vx, value.vy) + std::abs(value.wz) > 1e-10) {
        return;
      }
      fault_ = 0;
      state_ = {};
      state_.command.steering = measured_.steering;
      for (double & a : state_.command.steering) {
        a = std::clamp(a, -l_.steering, l_.steering);
      }
      state_.valid = true;
    }
    requested_ = value;
    command_stamp_ = t;
    command_wall_ = Steady::now();
    have_command_ = true;
    ++sequence_;
  }
  void feedback(const sensor_msgs::msg::JointState & msg)
  {
    observe_clock();
    const double t = stamp(msg.header.stamp);
    if (!valid_stamp(msg.header.stamp) || t > now().seconds() + 0.01 ||
        now().seconds() - t > feedback_timeout_) {
      stop(2);
      return;
    }
    if (have_feedback_ && t <= feedback_stamp_) {
      return;
    }  // Replays cannot refresh watchdog.
    core::Joints next;
    for (size_t i = 0; i < 4; ++i) {
      auto index = [&](const std::string & name) {
        if (std::count(msg.name.begin(), msg.name.end(), name) != 1) {
          return msg.name.size();
        }
        return static_cast<size_t>(std::find(msg.name.begin(), msg.name.end(), name) -
                                   msg.name.begin());
      };
      const size_t a = index(steering_names_[i]), w = index(wheel_names_[i]);
      if (a >= msg.name.size() || w >= msg.name.size() || a >= msg.position.size() ||
          w >= msg.velocity.size()) {
        stop(2);
        return;
      }
      next.steering[i] = msg.position[a];
      next.speed[i] = msg.velocity[w];
    }
    if (!core::finite(next)) {
      stop(2);
      return;
    }
    for (double a : next.steering) {
      if (std::abs(a) > l_.steering + 0.03) {
        stop(2);
        return;
      }
    }
    measured_ = next;
    const auto velocity = core::forward(g_, measured_);
    if (have_odom_ && t - feedback_stamp_ <= feedback_timeout_) {
      pose_ = core::integrate(pose_, velocity, t - feedback_stamp_);
    }
    feedback_stamp_ = t;
    feedback_time_ = msg.header.stamp;
    feedback_wall_ = Steady::now();
    have_feedback_ = have_odom_ = true;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.orientation.z = std::sin(pose_.yaw / 2);
    odom.pose.pose.orientation.w = std::cos(pose_.yaw / 2);
    odom.twist.twist = message(velocity);
    for (size_t i : {0, 7, 35}) {
      odom.pose.covariance[i] = pose_variance_;
      odom.twist.covariance[i] = twist_variance_;
    }
    for (size_t i : {14, 21, 28}) {
      odom.pose.covariance[i] = odom.twist.covariance[i] = 1e6;
    }
    if (odom_pub_) {
      odom_pub_->publish(odom);
    }
    if (tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = child_frame_;
      transform.transform.translation.x = pose_.x;
      transform.transform.translation.y = pose_.y;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_->sendTransform(transform);
    }
  }
  void publish_targets()
  {
    std_msgs::msg::Float64MultiArray wheels, steering;
    wheels.data.assign(state_.command.speed.begin(), state_.command.speed.end());
    steering.data.assign(state_.command.steering.begin(), state_.command.steering.end());
    wheel_pub_->publish(wheels);
    steering_pub_->publish(steering);
  }
  void tick()
  {
    observe_clock();
    double t = now().seconds(), dt = t - last_tick_;
    last_tick_ = t;
    if (!feedback_fresh()) {
      stop(2);
    } else if (!command_fresh()) {
      stop(1);
    }
    if (dt > 0.1) {
      stop(3);
    }
    if (!fault_ && dt > 0) {
      try {
        const auto out = generator_->step(requested_, measured_, state_, dt);
        state_ = out.state;
        execution_ = out.reason;
      } catch (const std::exception &) {
        stop(2);
      }
    }
    publish_targets();
    swerve_chassis_msgs::msg::ChassisState msg;
    msg.header.stamp = now();
    msg.header.frame_id = body_frame_;
    msg.feedback_stamp = feedback_time_;
    msg.command_stamp = rclcpp::Time(static_cast<int64_t>(command_stamp_ * 1e9));
    msg.command_sequence = sequence_;
    msg.steering_angles = measured_.steering;
    msg.wheel_speeds = measured_.speed;
    msg.target_steering = state_.command.steering;
    msg.target_wheel_speeds = state_.command.speed;
    msg.velocity = message(core::forward(g_, measured_));
    msg.requested_velocity = message(requested_);
    msg.executable_velocity = message(core::forward(g_, state_.command));
    msg.generator_valid = state_.valid;
    msg.execution = static_cast<uint8_t>(execution_);
    msg.fault = fault_;
    msg.command_fresh = command_fresh();
    msg.feedback_valid = feedback_fresh();
    msg.consistency_residual = core::residual(g_, measured_);
    msg.geometry = {g_.wheelbase, g_.track_width, g_.radius};
    msg.limits = {
        l_.steering,           l_.steering_rate, l_.wheel_speed,         l_.wheel_acceleration,
        l_.linear_speed,       l_.angular_speed, l_.linear_acceleration, l_.angular_acceleration,
        l_.tracking_tolerance, l_.stopped_speed};
    state_pub_->publish(msg);
  }
  core::Geometry g_;
  core::Limits l_;
  std::unique_ptr<core::Generator> generator_;
  core::State state_;
  core::Joints measured_;
  core::Twist requested_;
  core::Pose pose_;
  core::Reason execution_{core::Reason::stopped};
  uint8_t fault_{2};
  uint64_t sequence_{0};
  bool have_command_{false}, have_feedback_{false}, have_odom_{false};
  double cmd_timeout_{}, wall_timeout_{}, feedback_timeout_{}, pose_variance_{}, twist_variance_{};
  double command_stamp_{}, feedback_stamp_{}, last_tick_{}, last_clock_{};
  Steady::time_point command_wall_{}, feedback_wall_{};
  builtin_interfaces::msg::Time feedback_time_;
  std::string body_frame_, odom_frame_, child_frame_;
  std::vector<std::string> wheel_names_, steering_names_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_, steering_pub_;
  rclcpp::Publisher<swerve_chassis_msgs::msg::ChassisState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr legacy_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
namespace
{
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int)
{
  interrupted = 1;
}
}  // namespace
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  auto node = std::make_shared<Controller>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !interrupted) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  node->shutdown_stop();
  rclcpp::shutdown();
  return 0;
}
