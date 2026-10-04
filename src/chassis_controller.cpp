// SPDX-License-Identifier: Apache-2.0
#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "swerve_gazebo_sim/chassis_conversion.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "swerve_gazebo_sim/command_write.hpp"
#include <atomic>
#include <chrono>
#include <string>
#include <vector>

namespace swerve_gazebo_sim {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free &&
                  std::atomic<double>::is_always_lock_free,
              "endpoint requires lock-free 64-bit atomics");
class ChassisController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::vector<std::string>>("steering_joints", {});
      auto_declare<std::vector<std::string>>("wheel_joints", {});
      auto_declare<std::string>("body_frame", "base_footprint");
      auto_declare<std::string>("odom_frame", "odom");
      auto_declare<double>("wheelbase_m", .6);
      auto_declare<double>("track_m", .5);
      auto_declare<double>("wheel_radius_m", .1);
      auto_declare<double>("max_wheel_speed_mps", 2.0);
      auto_declare<double>("max_wheel_accel_mps2", 4.0);
      auto_declare<double>("max_steer_rate_radps", 2.5);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces_};
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces_};
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    const auto steering = get_node()->get_parameter("steering_joints").as_string_array();
    const auto wheels = get_node()->get_parameter("wheel_joints").as_string_array();
    if (steering.size() != 4 || wheels.size() != 4)
      return controller_interface::CallbackReturn::ERROR;
    interfaces_.clear();
    for (const auto &name : steering)
      interfaces_.push_back(name + "/position");
    for (const auto &name : wheels)
      interfaces_.push_back(name + "/velocity");
    auto unique = interfaces_;
    std::sort(unique.begin(), unique.end());
    if (std::adjacent_find(unique.begin(), unique.end()) != unique.end())
      return controller_interface::CallbackReturn::ERROR;
    core::Config config;
    config.wheelbase_m = get_node()->get_parameter("wheelbase_m").as_double();
    config.track_m = get_node()->get_parameter("track_m").as_double();
    config.wheel_radius_m = get_node()->get_parameter("wheel_radius_m").as_double();
    config.max_wheel_speed_mps = get_node()->get_parameter("max_wheel_speed_mps").as_double();
    config.max_wheel_accel_mps2 = get_node()->get_parameter("max_wheel_accel_mps2").as_double();
    config.max_steer_rate_radps = get_node()->get_parameter("max_steer_rate_radps").as_double();
    body_frame_ = get_node()->get_parameter("body_frame").as_string();
    odom_frame_ = get_node()->get_parameter("odom_frame").as_string();
    if (body_frame_.empty() || odom_frame_.empty() || body_frame_ == odom_frame_)
      return controller_interface::CallbackReturn::ERROR;
    try {
      // Fixed configuration after first configure; session history must survive
      // deactivate/cleanup/reconfigure. Restart the process to change the model.
      if (!runtime_) runtime_ = std::make_unique<ChassisRuntime>(config);
    } catch (const std::exception &) {
      return controller_interface::CallbackReturn::ERROR;
    }
    subscription_ = get_node()->create_subscription<msg::ChassisCommand>(
        "~/command", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const msg::ChassisCommand::SharedPtr message) {
          auto packet = decode(*message, body_frame_);
          packet.wall_s = wall_now();
          packet.receipt = ++receipt_;
          // A later valid packet cannot hide a cancellation or malformed packet
          // between two hardware updates.
          if (packet.malformed || !message->authorized) poison_ = true;
          buffer_.writeFromNonRT(std::make_shared<const ChassisPacket>(std::move(packet)));
        });
    context_subscription_ = get_node()->create_subscription<msg::ChassisContext>(
        "~/context", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const msg::ChassisContext::SharedPtr message) {
          auto context = decode(*message, odom_frame_);
          if (!context.valid) poison_ = true;
          context_.writeFromNonRT(std::make_shared<const ChassisContext>(std::move(context)));
        });
    status_publisher_ = get_node()->create_publisher<msg::ChassisState>("~/state", rclcpp::QoS(1));
    realtime_status_ = std::make_unique<realtime_tools::RealtimePublisher<msg::ChassisState>>(status_publisher_);
    publish_status_snapshot();
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    if (!runtime_ || command_interfaces_.size() != 8 || state_interfaces_.size() != 8)
      return controller_interface::CallbackReturn::ERROR;
    for (std::size_t i = 0; i < 8; ++i) {
      bool command_found = false, state_found = false;
      for (std::size_t j = 0; j < 8; ++j) {
        if (command_interfaces_[j].get_name() == interfaces_[i]) {
          command_index_[i] = j;
          command_found = true;
        }
        if (state_interfaces_[j].get_name() == interfaces_[i]) {
          state_index_[i] = j;
          state_found = true;
        }
      }
      if (!command_found || !state_found)
        return controller_interface::CallbackReturn::ERROR;
    }
    runtime_->stop();
    // Discard commands received while inactive. An explicit new arm is
    // required.
    activation_receipt_ = receipt_.load();
    const double sim = get_node()->now().seconds();
    const bool written = write(runtime_->update(sim, wall_now(), measured(), {}));
    if (!written)
      runtime_->stop(EndpointFault::WriteFailure);
    publish_status_snapshot();
    if (!written)
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    runtime_->stop();
    const double sim = get_node()->now().seconds();
    const bool written = write(runtime_->update(sim, wall_now(), measured(), {}));
    if (!written)
      runtime_->stop(EndpointFault::WriteFailure);
    publish_status_snapshot();
    if (!written)
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &) override {
    const auto packet = *buffer_.readFromRT();
    const auto context = *context_.readFromRT();
    if (poison_.exchange(false)) {
      runtime_->stop(EndpointFault::CommandRejected);
      activation_receipt_ = receipt_.load();
    }
    try {
      const auto targets = runtime_->update(time.seconds(), wall_now(), measured(),
          context ? *context : ChassisContext{},
          packet && packet->receipt > activation_receipt_ ? std::optional<ChassisPacket>(*packet)
                                                         : std::nullopt);
      if (!write(targets)) {
        runtime_->stop(EndpointFault::WriteFailure);
        write(runtime_->update(time.seconds(), wall_now(), measured(), {}));
      }
    } catch (const std::exception &e) {
      runtime_->stop(EndpointFault::CommandRejected);
      EndpointTargets stopped = measured();
      stopped.wheels.fill(0);
      for (double &a : stopped.steering) a = std::isfinite(a) ? std::clamp(a, -1.5707963267948966, 1.5707963267948966) : 0;
      write(stopped);
      (void)e;
    }
    publish_status_snapshot();
    return controller_interface::return_type::OK;
  }

private:
  void publish_status_snapshot() {
    if (realtime_status_ && realtime_status_->trylock()) {
      realtime_status_->msg_ = encode(*runtime_, odom_frame_, body_frame_);
      realtime_status_->unlockAndPublish();
    }
  }
  static double wall_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  EndpointTargets measured() const {
    EndpointTargets state;
    for (std::size_t i = 0; i < 4; ++i) {
      state.steering[i] = state_interfaces_[state_index_[i]].get_value();
      state.wheels[i] = state_interfaces_[state_index_[i + 4]].get_value();
    }
    return state;
  }
  bool write(const EndpointTargets &targets) {
    return write_joint_targets(command_interfaces_, command_index_, targets);
  }
  std::vector<std::string> interfaces_;
  std::array<std::size_t, 8> command_index_{}, state_index_{};
  std::unique_ptr<ChassisRuntime> runtime_;
  std::string body_frame_, odom_frame_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<const ChassisPacket>> buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<const ChassisContext>> context_;
  rclcpp::Subscription<msg::ChassisCommand>::SharedPtr subscription_;
  rclcpp::Subscription<msg::ChassisContext>::SharedPtr context_subscription_;
  rclcpp::Publisher<msg::ChassisState>::SharedPtr status_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<msg::ChassisState>> realtime_status_;
  std::atomic<std::uint64_t> receipt_{0};
  std::atomic<bool> poison_{false};
  std::uint64_t activation_receipt_ = 0;

};
} // namespace swerve_gazebo_sim
PLUGINLIB_EXPORT_CLASS(swerve_gazebo_sim::ChassisController,
                       controller_interface::ControllerInterface)
