// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "gz_ros2_control/gz_system_interface.hpp"
#ifdef SWERVE_GAZEBO_HUMBLE
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#else
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#endif

namespace swerve_gazebo_sim {
using PhysicsTime = sim::components::Component<double, class PhysicsTimeTag>;
GZ_SIM_REGISTER_COMPONENT("swerve_gazebo_sim.PhysicsTime", PhysicsTime)
} // namespace swerve_gazebo_sim
