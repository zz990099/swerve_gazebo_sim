// SPDX-License-Identifier: Apache-2.0
#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "swerve_gazebo_sim/command_write.hpp"
#include <atomic>
#include <chrono>
#include <string>
#include <vector>

namespace swerve_gazebo_sim {
static_assert(std::atomic<std::uint64_t>::is_always_lock_free &&
                  std::atomic<double>::is_always_lock_free,
              "endpoint requires lock-free 64-bit atomics");
class GuardedJointController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::vector<std::string>>("steering_joints", {});
      auto_declare<std::vector<std::string>>("wheel_joints", {});
      auto_declare<double>("wall_timeout_s", .1);
      auto_declare<double>("max_validity_s", .03);
      auto_declare<double>("max_wheel_speed_radps", 20);
      auto_declare<double>("steering_limit_rad", 1.5707963267948966);
      auto_declare<double>("stopped_wheel_radps", .05);
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
    EndpointLimits limits;
    limits.wall_timeout_s = get_node()->get_parameter("wall_timeout_s").as_double();
    limits.max_validity_s = get_node()->get_parameter("max_validity_s").as_double();
    limits.max_wheel_speed_radps = get_node()->get_parameter("max_wheel_speed_radps").as_double();
    limits.steering_limit_rad = get_node()->get_parameter("steering_limit_rad").as_double();
    limits.stopped_wheel_radps = get_node()->get_parameter("stopped_wheel_radps").as_double();
    try {
      if (!guard_)
        guard_.emplace(limits);
    } catch (const std::exception &) {
      return controller_interface::CallbackReturn::ERROR;
    }
    publish_status_snapshot(get_node()->now().seconds());
    subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
          EndpointPacket packet;
          packet.received_wall_s = wall_now();
          packet.receipt = ++receipt_;
          packet.malformed = message->data.size() != packet.data.size() ||
                             !message->layout.dim.empty() || message->layout.data_offset != 0;
          if (!packet.malformed)
            std::copy(message->data.begin(), message->data.end(), packet.data.begin());
          buffer_.writeFromNonRT(packet);
        });
    status_publisher_ =
        get_node()->create_publisher<std_msgs::msg::UInt64MultiArray>("~/status", 1);
    status_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(10), [this]() {
      // Allocation and ROS publication stay outside the controller update
      // thread.
      for (int attempt = 0; attempt < 3; ++attempt) {
        const auto version = status_version_.load();
        if (version % 2)
          continue;
        std_msgs::msg::UInt64MultiArray msg;
        msg.data = {status_session_.load(), status_sequence_.load(), status_fault_.load(),
                    status_reason_.load()};
        const double sim = status_sim_.load(), source = status_source_.load(),
                     deadline = status_deadline_.load(), age = status_wall_age_.load();
        if (version == status_version_.load()) {
          status_publisher_->publish(msg);
          if (msg.data[2] && msg.data[3] != logged_reason_) {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Endpoint fault reason=%llu sim=%.9f sample=%.9f "
                        "deadline=%.9f wall_age=%.6f",
                        static_cast<unsigned long long>(msg.data[3]), sim, source, deadline, age);
          }
          logged_reason_ = msg.data[2] ? msg.data[3] : 0;
          break;
        }
      }
    });
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    if (!guard_ || command_interfaces_.size() != 8 || state_interfaces_.size() != 8)
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
    guard_->deactivate();
    // Discard commands received while inactive. An explicit new arm is
    // required.
    activation_receipt_ = receipt_.load();
    const double sim = get_node()->now().seconds();
    const bool written = write(guard_->update(sim, wall_now(), measured()));
    if (!written)
      guard_->deactivate(EndpointFault::WriteFailure);
    publish_status_snapshot(sim);
    if (!written)
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    guard_->deactivate();
    const double sim = get_node()->now().seconds();
    const bool written = write(guard_->update(sim, wall_now(), measured()));
    if (!written)
      guard_->deactivate(EndpointFault::WriteFailure);
    publish_status_snapshot(sim);
    if (!written)
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &) override {
    const auto packet = *buffer_.readFromRT();
    const auto targets =
        guard_->update(time.seconds(), wall_now(), measured(),
                       packet.receipt > activation_receipt_ ? std::optional<EndpointPacket>(packet)
                                                            : std::nullopt);
    if (!write(targets)) {
      guard_->deactivate(EndpointFault::WriteFailure);
      // Retry stopped targets immediately and on every subsequent faulted tick.
      write(guard_->update(time.seconds(), wall_now(), measured()));
    }
    publish_status_snapshot(time.seconds(), &packet);
    // A guard fault is a safe command, not permission to release the
    // interfaces.
    return controller_interface::return_type::OK;
  }

private:
  // Lifecycle callbacks and update are serialized by the controller manager.
  // The timer reads atomics only; it must never touch the live guard state.
  void publish_status_snapshot(double sim, const EndpointPacket *packet = nullptr) {
    ++status_version_;
    status_session_ = guard_->session();
    status_sequence_ = guard_->sequence();
    status_fault_ = guard_->fault() ? 1 : 0;
    status_reason_ = static_cast<std::uint64_t>(guard_->fault_reason());
    status_sim_ = sim;
    status_source_ = packet ? packet->data[3] : 0;
    status_deadline_ = packet ? packet->data[4] : 0;
    status_wall_age_ = packet && packet->receipt ? wall_now() - packet->received_wall_s : -1;
    ++status_version_;
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
  std::optional<EndpointGuard> guard_;
  realtime_tools::RealtimeBuffer<EndpointPacket> buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::atomic<std::uint64_t> receipt_{0};
  std::uint64_t activation_receipt_ = 0;
  std::atomic<std::uint64_t> status_version_{0}, status_session_{0}, status_sequence_{0},
      status_fault_{1}, status_reason_{1};
  std::atomic<double> status_sim_{0}, status_source_{0}, status_deadline_{0}, status_wall_age_{-1};
  std::uint64_t logged_reason_ = 0; // Only the non-real-time timer touches this.
};
} // namespace swerve_gazebo_sim
PLUGINLIB_EXPORT_CLASS(swerve_gazebo_sim::GuardedJointController,
                       controller_interface::ControllerInterface)
