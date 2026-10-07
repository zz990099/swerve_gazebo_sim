// SPDX-License-Identifier: Apache-2.0
#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "swerve_gazebo_sim/command_write.hpp"
#include "swerve_gazebo_sim/msg/chassis_command.hpp"
#include "swerve_gazebo_sim/msg/chassis_state.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
namespace swerve_gazebo_sim {
class ChassisController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::vector<std::string>>("steering_joints", {});
      auto_declare<std::vector<std::string>>("wheel_joints", {});
      auto_declare<std::string>("body_frame", "base_footprint");
      auto_declare<std::string>("simulation_time_interface",
                                "execution_clock/simulation_time");
      auto_declare<std::int64_t>("update_rate", 100);
      Config c;
#define FIELD(name) auto_declare<double>(#name, c.name);
      FIELD(wheelbase)
      FIELD(track_width)
      FIELD(wheel_radius)
      FIELD(steering_limit)
      FIELD(max_wheel_speed)
      FIELD(max_wheel_acceleration)
      FIELD(max_steering_rate)
      FIELD(steering_alignment_tolerance)
      FIELD(steering_alignment_duration)
      FIELD(mode_switch_timeout)
      FIELD(stopped_wheel_speed)
      FIELD(cmd_timeout)
      FIELD(wall_timeout)
      FIELD(drive_steering_limit)
      FIELD(max_linear_speed)
      FIELD(max_angular_speed)
      FIELD(max_linear_acceleration)
      FIELD(max_angular_acceleration)
#undef FIELD
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            names_};
  }
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override {
    auto n = names_;
    n.push_back(clock_);
    return {controller_interface::interface_configuration_type::INDIVIDUAL, n};
  }
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override {
    try {
      std::vector<std::string> names;
      for (const auto &kind : {"steering", "wheel"}) {
        auto joints = get_node()
                          ->get_parameter(std::string(kind) + "_joints")
                          .as_string_array();
        if (joints.size() != 4)
          throw std::invalid_argument("Four joints per kind required");
        for (const auto &j : joints) {
          if (j.empty())
            throw std::invalid_argument("Empty joint");
          names.push_back(j + (std::string(kind) == "steering" ? "/position"
                                                               : "/velocity"));
        }
      }
      auto unique = names;
      std::sort(unique.begin(), unique.end());
      if (std::adjacent_find(unique.begin(), unique.end()) != unique.end())
        throw std::invalid_argument("Duplicate joints");
      Config c;
      std::vector<double> parameters;
#define FIELD(name)                                                            \
  c.name = get_node()->get_parameter(#name).as_double();                       \
  parameters.push_back(c.name);
      FIELD(wheelbase)
      FIELD(track_width)
      FIELD(wheel_radius)
      FIELD(steering_limit)
      FIELD(max_wheel_speed)
      FIELD(max_wheel_acceleration)
      FIELD(max_steering_rate)
      FIELD(steering_alignment_tolerance)
      FIELD(steering_alignment_duration)
      FIELD(mode_switch_timeout)
      FIELD(stopped_wheel_speed)
      FIELD(cmd_timeout)
      FIELD(wall_timeout)
      FIELD(drive_steering_limit)
      FIELD(max_linear_speed)
      FIELD(max_angular_speed)
      FIELD(max_linear_acceleration)
      FIELD(max_angular_acceleration)
#undef FIELD
      c.update_rate = static_cast<double>(get_node()->get_parameter("update_rate").as_int());
      parameters.push_back(c.update_rate);
      validate(c);
      auto body = get_node()->get_parameter("body_frame").as_string();
      auto clock =
          get_node()->get_parameter("simulation_time_interface").as_string();
      if (body.empty() || clock.empty())
        throw std::invalid_argument("Frame and clock required");
      if (runtime_ && (names != names_ || body != body_ || clock != clock_ ||
                       parameters != parameters_))
        throw std::invalid_argument("Restart to change chassis configuration");
      names_ = names;
      body_ = body;
      clock_ = clock;
      parameters_ = parameters;
      if (!runtime_)
        runtime_ = std::make_unique<ChassisRuntime>(c);
      subscription_ = get_node()->create_subscription<msg::ChassisCommand>(
          "~/command", rclcpp::QoS(1).reliable().durability_volatile(),
          [this](msg::ChassisCommand::SharedPtr m) {
            auto p = std::make_shared<Command>();
            p->receipt = ++receipt_;
            p->wall = wall();
            p->session = m->session_id;
            p->sequence = m->sequence;
            p->request = m->request_id;
            p->reset = m->reset;
            p->mode = static_cast<Mode>(m->mode);
            p->stamp = m->header.stamp.sec + 1e-9 * m->header.stamp.nanosec;
            p->velocity = twist(m->velocity);
            p->entry = twist(m->entry_velocity);
            p->valid =
                m->header.frame_id == body_ && m->header.stamp.sec >= 0 &&
                m->header.stamp.nanosec < 1000000000 && planar(m->velocity) &&
                planar(m->entry_velocity) && permitted(p->mode, p->velocity) &&
                permitted(p->mode, p->entry);
            if (!p->valid)
              poison_ = true;
            buffer_.writeFromNonRT(std::move(p));
          });
      publisher_ = get_node()->create_publisher<msg::ChassisState>(
          "~/state", rclcpp::QoS(1).best_effort().durability_volatile());
      realtime_ = std::make_unique<
          realtime_tools::RealtimePublisher<msg::ChassisState>>(publisher_);
      return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    if (!runtime_ || command_interfaces_.size() != 8 ||
        state_interfaces_.size() != 9)
      return controller_interface::CallbackReturn::ERROR;
    for (std::size_t i = 0; i < 8; ++i) {
      bool a = false, b = false;
      for (std::size_t j = 0; j < 8; ++j)
        if (command_interfaces_[j].get_name() == names_[i]) {
          commands_[i] = j;
          a = true;
        }
      for (std::size_t j = 0; j < 9; ++j)
        if (state_interfaces_[j].get_name() == names_[i]) {
          states_[i] = j;
          b = true;
        }
      if (!a || !b)
        return controller_interface::CallbackReturn::ERROR;
    }
    bool found = false;
    for (std::size_t j = 0; j < 9; ++j)
      if (state_interfaces_[j].get_name() == clock_) {
        clock_index_ = j;
        found = true;
      }
    if (!found)
      return controller_interface::CallbackReturn::ERROR;
    activation_receipt_ = receipt_;
    runtime_->stop();
    return write_joint_targets(command_interfaces_, commands_,
                               runtime_->stopped_targets(measured()))
               ? controller_interface::CallbackReturn::SUCCESS
               : controller_interface::CallbackReturn::ERROR;
  }
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    runtime_->stop();
    if (!write_joint_targets(command_interfaces_, commands_,
                             runtime_->stopped_targets(measured())))
      runtime_->stop(Fault::Write);
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override {
    const auto feedback = measured();
    const double now = state_interfaces_[clock_index_].get_value();
    auto p = *buffer_.readFromRT();
    if (poison_.exchange(false)) {
      runtime_->stop(Fault::Command);
      activation_receipt_ = receipt_;
    }
    auto targets = runtime_->update(
        now, wall(), feedback,
        p && p->receipt > activation_receipt_ ? p.get() : nullptr);
    if (!write_joint_targets(command_interfaces_, commands_, targets)) {
      runtime_->stop(Fault::Write);
      write_joint_targets(command_interfaces_, commands_,
                          runtime_->stopped_targets(feedback));
    }
    if (realtime_->trylock()) {
      auto &m = realtime_->msg_;
      m.header.frame_id = body_;
      // Invalid clocks are faults; never convert NaN/overflow into integer
      // stamps.
      m.header.stamp = builtin_interfaces::msg::Time{};
      if (std::isfinite(now) && now >= 0 && now < 2147483647.) {
        const auto ns = static_cast<std::int64_t>(std::llround(now * 1e9));
        m.header.stamp.sec = static_cast<std::int32_t>(ns / 1000000000);
        m.header.stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000);
      }
      m.session_id = runtime_->session();
      m.sequence = runtime_->sequence();
      m.request_id = runtime_->request();
      m.actual_mode = static_cast<std::uint8_t>(runtime_->actual_mode());
      m.requested_mode = static_cast<std::uint8_t>(runtime_->requested_mode());
      m.phase = static_cast<std::uint8_t>(runtime_->phase());
      m.fault = static_cast<std::uint8_t>(runtime_->fault());
      m.confirmed = runtime_->confirmed();
      m.steering_angles = feedback.steering;
      m.wheel_speeds = feedback.wheels;
      m.velocity = encode(runtime_->forward(feedback));
      m.accepted_entry_velocity = encode(runtime_->entry());
      m.accepted_steering = runtime_->entry_angles();
      realtime_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }

private:
  static Twist twist(const geometry_msgs::msg::Twist &t) {
    return {t.linear.x, t.linear.y, t.angular.z};
  }
  static bool planar(const geometry_msgs::msg::Twist &t) {
    return t.linear.z == 0 && t.angular.x == 0 && t.angular.y == 0;
  }
  static geometry_msgs::msg::Twist encode(Twist t) {
    geometry_msgs::msg::Twist m;
    m.linear.x = t.vx;
    m.linear.y = t.vy;
    m.angular.z = t.wz;
    return m;
  }
  static double wall() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  JointTargets measured() const {
    JointTargets s;
    for (std::size_t i = 0; i < 4; ++i) {
      s.steering[i] = state_interfaces_[states_[i]].get_value();
      s.wheels[i] = state_interfaces_[states_[i + 4]].get_value();
    }
    return s;
  }
  std::unique_ptr<ChassisRuntime> runtime_;
  std::vector<std::string> names_;
  std::string body_, clock_;
  std::vector<double> parameters_;
  std::array<std::size_t, 8> commands_{}, states_{};
  std::size_t clock_index_ = 0;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Command>> buffer_;
  std::atomic<std::uint64_t> receipt_{0};
  std::atomic<bool> poison_{false};
  std::uint64_t activation_receipt_ = 0;
  rclcpp::Subscription<msg::ChassisCommand>::SharedPtr subscription_;
  rclcpp::Publisher<msg::ChassisState>::SharedPtr publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<msg::ChassisState>>
      realtime_;
};
} // namespace swerve_gazebo_sim
PLUGINLIB_EXPORT_CLASS(swerve_gazebo_sim::ChassisController,
                       controller_interface::ControllerInterface)
