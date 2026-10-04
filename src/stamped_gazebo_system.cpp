// SPDX-License-Identifier: Apache-2.0
#include "gz_ros2_control/gz_system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include <limits>

namespace swerve_gazebo_sim {
// Delegate joint physics to the upstream plugin. Export the authoritative time
// from the same synchronous read; controller-manager ROS clocks may lag/jitter.
class StampedGazeboSystem : public gz_ros2_control::GazeboSimSystemInterface {
public:
  StampedGazeboSystem()
      : loader_("gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface"),
        joints_(
            loader_.createSharedInstance("gz_ros2_control/GazeboSimSystem")) {}
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    auto native = info;
#ifdef SWERVE_GAZEBO_HUMBLE
    native.hardware_class_type = "gz_ros2_control/GazeboSimSystem";
#else
    native.hardware_plugin_name = "gz_ros2_control/GazeboSimSystem";
#endif
    const auto clock = info.hardware_parameters.find("clock_name");
    if (clock == info.hardware_parameters.end() || clock->second.empty())
      return hardware_interface::CallbackReturn::ERROR;
    clock_name_ = clock->second;
    return joints_->on_init(native);
  }
  bool initSim(rclcpp::Node::SharedPtr &node,
               std::map<std::string, sim::Entity> &joints,
               const hardware_interface::HardwareInfo &info,
               sim::EntityComponentManager &ecm,
#ifdef SWERVE_GAZEBO_HUMBLE
               int &rate
#else
               unsigned int rate
#endif
               ) override {
    return joints_->initSim(node, joints, info, ecm, rate);
  }
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    auto interfaces = joints_->export_state_interfaces();
    interfaces.emplace_back(clock_name_, "simulation_time", &simulation_time_);
    return interfaces;
  }
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    return joints_->export_command_interfaces();
  }
  hardware_interface::return_type
  read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    const auto result = joints_->read(time, period);
    simulation_time_ = result == hardware_interface::return_type::OK
                           ? time.seconds()
                           : std::numeric_limits<double>::quiet_NaN();
    return result;
  }
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    return joints_->write(time, period);
  }
  hardware_interface::return_type
  prepare_command_mode_switch(const std::vector<std::string> &start,
                              const std::vector<std::string> &stop) override {
    return joints_->prepare_command_mode_switch(start, stop);
  }
  hardware_interface::return_type
  perform_command_mode_switch(const std::vector<std::string> &start,
                              const std::vector<std::string> &stop) override {
    return joints_->perform_command_mode_switch(start, stop);
  }
#define FORWARD_LIFECYCLE(method)                                              \
  hardware_interface::CallbackReturn method(                                   \
      const rclcpp_lifecycle::State &state) override {                         \
    return joints_->method(state);                                             \
  }
  FORWARD_LIFECYCLE(on_configure)
  FORWARD_LIFECYCLE(on_cleanup)
  FORWARD_LIFECYCLE(on_activate)
  FORWARD_LIFECYCLE(on_deactivate)
  FORWARD_LIFECYCLE(on_shutdown)
  FORWARD_LIFECYCLE(on_error)
#undef FORWARD_LIFECYCLE
private:
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> loader_;
  std::shared_ptr<gz_ros2_control::GazeboSimSystemInterface> joints_;
  std::string clock_name_;
  double simulation_time_ = std::numeric_limits<double>::quiet_NaN();
};
} // namespace swerve_gazebo_sim
PLUGINLIB_EXPORT_CLASS(swerve_gazebo_sim::StampedGazeboSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
