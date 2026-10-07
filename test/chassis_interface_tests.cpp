// SPDX-License-Identifier: Apache-2.0
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swerve_gazebo_sim/msg/chassis_command.hpp"
#include "swerve_gazebo_sim/msg/chassis_state.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
using namespace swerve_gazebo_sim;
using Controller = controller_interface::ControllerInterface;
void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
template <class C>
auto initialize(C &c, const rclcpp::NodeOptions &o, int)
    -> decltype(c.init("chassis_test", "", o)) {
  return c.init("chassis_test", "", o);
}
template <class C>
auto initialize(C &c, const rclcpp::NodeOptions &o, long)
    -> decltype(c.init("chassis_test", "", 100, "", o)) {
  return c.init("chassis_test", "", 100, "", o);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    std::array<double, 8> commands{}, states{};
    std::vector<hardware_interface::CommandInterface> ch;
    std::vector<hardware_interface::StateInterface> sh;
    ch.reserve(8);
    sh.reserve(9);
    double physics_time = 0;
    pluginlib::ClassLoader<Controller> loader(
        "controller_interface", "controller_interface::ControllerInterface");
    auto controller =
        loader.createSharedInstance("swerve_gazebo_sim/ChassisController");
    rclcpp::NodeOptions opts;
    opts.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    require(initialize(*controller, opts, 0) ==
                controller_interface::return_type::OK,
            "init failed");
    std::vector<std::string> steering{"fl_steering_joint", "fr_steering_joint",
                                      "rl_steering_joint", "rr_steering_joint"};
    std::vector<std::string> wheels{"fl_wheel_joint", "fr_wheel_joint",
                                    "rl_wheel_joint", "rr_wheel_joint"};
    controller->get_node()->set_parameter(
        rclcpp::Parameter("steering_joints", steering));
    controller->get_node()->set_parameter(
        rclcpp::Parameter("wheel_joints", wheels));
    controller->get_node()->set_parameter(rclcpp::Parameter("update_rate", 100));
    require(controller->configure().label() == "inactive", "configure failed");
    std::vector<hardware_interface::LoanedCommandInterface> cl;
    std::vector<hardware_interface::LoanedStateInterface> sl;
    for (std::size_t i = 0; i < 8; ++i) {
      auto name = i < 4 ? steering[i] : wheels[i - 4];
      auto kind = i < 4 ? "position" : "velocity";
      ch.emplace_back(name, kind, &commands[i]);
      sh.emplace_back(name, kind, &states[i]);
      cl.emplace_back(ch.back());
      sl.emplace_back(sh.back());
    }
    sh.emplace_back("execution_clock", "simulation_time", &physics_time);
    sl.emplace_back(sh.back());
    controller->assign_interfaces(std::move(cl), std::move(sl));
    require(controller->get_node()->activate().label() == "active",
            "activate failed");
    auto observer = std::make_shared<rclcpp::Node>("chassis_observer");
    auto pub = observer->create_publisher<msg::ChassisCommand>(
        "/chassis_test/command", 1);
    msg::ChassisState status;
    int count = 0;
    auto sub = observer->create_subscription<msg::ChassisState>(
        "/chassis_test/state", rclcpp::QoS(1).best_effort(),
        [&](msg::ChassisState::SharedPtr s) {
          status = *s;
          ++count;
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    executor.add_node(controller->get_node()->get_node_base_interface());
    auto spin = [&](int ms) {
      auto end =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
      do {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } while (std::chrono::steady_clock::now() < end);
    };
    for (int i = 0; i < 100 && pub->get_subscription_count() != 1; ++i)
      spin(10);
    require(pub->get_subscription_count() == 1, "DDS discovery failed");
    auto update = [&](double now) {
      physics_time = now;
      controller->update(
          rclcpp::Time(static_cast<int64_t>(std::llround((now - .005) * 1e9)),
                       RCL_ROS_TIME),
          rclcpp::Duration::from_seconds(.01));
    };
    update(.99);
    update(1);
    msg::ChassisCommand m;
    m.header.frame_id = "base_footprint";
    m.header.stamp.sec = 1;
    m.session_id = 1;
    m.sequence = 1;
    m.reset = true;
    pub->publish(m);
    spin(10);
    update(1);
    spin(10);
    require(count > 0 && status.confirmed && status.session_id == 1,
            "Stopped reset failed");
    m.reset = false;
    m.velocity.linear.x = .2;
    for (int i = 1; i <= 15; ++i) {
      ++m.sequence;
      m.header.stamp.nanosec = i * 10000000;
      pub->publish(m);
      spin(5);
      states = commands;
      update(1 + i * .01);
      spin(2);
    }
    require(status.confirmed && commands[4] > 0, "Drive failed");
    // An invalid callback cannot be hidden by a later valid buffer overwrite.
    m.header.frame_id = "map";
    pub->publish(m);
    spin(10);
    m.header.frame_id = "base_footprint";
    ++m.sequence;
    pub->publish(m);
    spin(10);
    update(1.16);
    spin(10);
    require(status.fault != 0, "Malformed frame was overwritten");
    for (std::size_t i = 4; i < 8; ++i)
      require(commands[i] == 0, "Fault did not zero wheels");
    states = commands;
    m.reset = true;
    m.velocity.linear.x = 0;
    ++m.sequence;
    pub->publish(m);
    spin(10);
    update(1.17);
    spin(10);
    require(status.fault != 0, "Old session rearmed");
    m.session_id = 2;
    ++m.sequence;
    pub->publish(m);
    spin(10);
    update(1.18);
    spin(10);
    require(status.confirmed && status.session_id == 2,
            "Fresh recovery failed");
    controller->get_node()->deactivate();
    controller->release_interfaces();
    require(controller->get_node()->cleanup().label() == "unconfigured",
            "cleanup failed");
    require(controller->configure().label() == "inactive",
            "unchanged reconfiguration failed");
    require(controller->get_node()->cleanup().label() == "unconfigured",
            "second cleanup failed");
    controller->get_node()->set_parameter(rclcpp::Parameter("track_width", .7));
    require(controller->configure().label() != "inactive",
            "reconfiguration silently changed the execution model");
    std::cout << "Typed command/state, physics clock, fault and session checks "
                 "passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
