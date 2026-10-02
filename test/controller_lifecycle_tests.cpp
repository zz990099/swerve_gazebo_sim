// SPDX-License-Identifier: Apache-2.0
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include <array>
#include <chrono>
#include <iostream>
#include <thread>

using Controller = controller_interface::ControllerInterface;
void require(bool ok, const char *reason) {
  if (!ok)
    throw std::runtime_error(reason);
}
// The released Humble and Jazzy ControllerInterface init signatures differ.
template <class C>
auto initialize(C &c, const rclcpp::NodeOptions &options, int)
    -> decltype(c.init("guarded_lifecycle", "", options)) {
  return c.init("guarded_lifecycle", "", options);
}
template <class C>
auto initialize(C &c, const rclcpp::NodeOptions &options, long)
    -> decltype(c.init("guarded_lifecycle", "", 100, "", options)) {
  return c.init("guarded_lifecycle", "", 100, "", options);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // Loaned interfaces in the plugin must be destroyed before these handles,
    // including when a regression throws before explicit release_interfaces.
    std::array<double, 8> commands{}, states{};
    std::vector<hardware_interface::CommandInterface> command_handles;
    std::vector<hardware_interface::StateInterface> state_handles;
    pluginlib::ClassLoader<Controller> loader("controller_interface",
                                              "controller_interface::ControllerInterface");
    auto controller = loader.createSharedInstance("swerve_gazebo_sim/GuardedJointController");
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    require(initialize(*controller, options, 0) == controller_interface::return_type::OK,
            "plugin initialization failed");
    const std::vector<std::string> steering{"fl_steering_joint", "fr_steering_joint",
                                            "rl_steering_joint", "rr_steering_joint"};
    const std::vector<std::string> wheels{"fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint",
                                          "rr_wheel_joint"};
    controller->get_node()->set_parameter(rclcpp::Parameter("steering_joints", steering));
    controller->get_node()->set_parameter(rclcpp::Parameter("wheel_joints", wheels));
    require(controller->configure().label() == "inactive", "configure failed");

    command_handles.reserve(8);
    state_handles.reserve(8);
    for (std::size_t i = 0; i < 8; ++i) {
      const auto &name = i < 4 ? steering[i] : wheels[i - 4];
      const std::string kind = i < 4 ? "position" : "velocity";
      command_handles.emplace_back(name, kind, &commands[i]);
      state_handles.emplace_back(name, kind, &states[i]);
    }
    auto assign = [&]() {
      std::vector<hardware_interface::LoanedCommandInterface> command_loans;
      std::vector<hardware_interface::LoanedStateInterface> state_loans;
      for (std::size_t i = 0; i < 8; ++i) {
        command_loans.emplace_back(command_handles[i]);
        state_loans.emplace_back(state_handles[i]);
      }
      controller->assign_interfaces(std::move(command_loans), std::move(state_loans));
    };
    assign();
    auto observer = std::make_shared<rclcpp::Node>("guarded_lifecycle_observer");
    std::vector<std::uint64_t> status;
    std::size_t messages = 0;
    auto subscription = observer->create_subscription<std_msgs::msg::UInt64MultiArray>(
        "/guarded_lifecycle/status", 10, [&](std_msgs::msg::UInt64MultiArray::SharedPtr msg) {
          status = msg->data;
          ++messages;
        });
    auto publisher = observer->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/guarded_lifecycle/commands", rclcpp::QoS(1).best_effort());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller->get_node()->get_node_base_interface());
    executor.add_node(observer);
    const auto wait = [&](const auto &predicate, bool updating = false) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
      while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        if (updating)
          controller->update(controller->get_node()->now(), rclcpp::Duration::from_seconds(.01));
        if (predicate())
          return;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      throw std::runtime_error("lifecycle status/transport deadline exceeded");
    };
    wait([&]() { return publisher->get_subscription_count() == 1 && status.size() == 4; });
    require(status[2] == 1 && status[3] == 1, "configured controller must report disarmed");
    require(controller->get_node()->activate().label() == "active", "activate failed");
    auto arm = [&](std::uint64_t session) {
      std_msgs::msg::Float64MultiArray message;
      const double sim = controller->get_node()->now().seconds();
      message.data = {
          static_cast<double>(session), 1, 1, sim, sim + .03, 0, 0, 0, 0, 0, 0, 0, 0, 1};
      const auto old_count = messages;
      wait(
          [&]() {
            // Frozen retransmissions do not refresh guard deadlines.
            publisher->publish(message);
            return messages > old_count && status[0] == session && status[2] == 0;
          },
          true);
    };
    arm(1);
    std_msgs::msg::Float64MultiArray drive;
    const double drive_time = controller->get_node()->now().seconds();
    drive.data = {1, 2, 0, drive_time, drive_time + .03, 0, 0, 0, 0, 2, 2, 2, 2, 1};
    const auto drive_count = messages;
    publisher->publish(drive);
    wait([&]() { return messages > drive_count && status[1] == 2 && status[2] == 0; }, true);
    for (std::size_t i = 4; i < 8; ++i)
      require(commands[i] == 2, "test must stop previously nonzero wheel commands");
    require(controller->get_node()->deactivate().label() == "inactive", "deactivate failed");
    const auto stopped_count = messages;
    // Deliberately NO update calls: an inactive controller's timer must publish
    // the lifecycle snapshot, rather than repeatedly sending its old healthy
    // ACK.
    wait([&]() { return messages >= stopped_count + 3 && status[2] == 1 && status[3] == 1; });
    require(status[0] == 1 && status[1] == 2, "deactivation must preserve accepted history");
    for (std::size_t i = 4; i < 8; ++i)
      require(commands[i] == 0, "deactivation must zero every wheel command");
    const auto activation_count = messages;
    require(controller->get_node()->activate().label() == "active", "reactivate failed");
    wait([&]() { return messages > activation_count && status[2] == 1 && status[3] == 1; });
    arm(2);
    require(controller->get_node()->deactivate().label() == "inactive", "second stop failed");
    controller->release_interfaces();
    require(controller->get_node()->cleanup().label() == "unconfigured", "cleanup failed");
    const auto configure_count = messages;
    require(controller->configure().label() == "inactive", "reconfigure failed");
    wait([&]() { return messages > configure_count && status[0] == 2 && status[2] == 1; });
    assign();
    require(controller->get_node()->activate().label() == "active", "final activate failed");
    arm(3);
    controller->get_node()->deactivate();
    controller->release_interfaces();
    std::cout << "ROS plugin lifecycle status, stop, reactivation and history "
                 "passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
