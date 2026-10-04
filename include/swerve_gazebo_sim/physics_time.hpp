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
#ifdef SWERVE_GAZEBO_HUMBLE
IGN_GAZEBO_REGISTER_COMPONENT("swerve_gazebo_sim.PhysicsTime", PhysicsTime)
#else
GZ_SIM_REGISTER_COMPONENT("swerve_gazebo_sim.PhysicsTime", PhysicsTime)
#endif
} // namespace swerve_gazebo_sim
