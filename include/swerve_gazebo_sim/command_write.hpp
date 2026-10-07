// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "swerve_gazebo_sim/chassis_runtime.hpp"
#include <type_traits>

namespace swerve_gazebo_sim {
// Humble setters return void, newer ros2_control setters can report failure.
// Attempt EVERY interface even after one failure; the caller must latch a fault
// and retry zero wheel targets before reporting a healthy update.
template <class Interfaces>
bool write_joint_targets(Interfaces &interfaces,
                         const std::array<std::size_t, 8> &indices,
                         const JointTargets &targets) {
  auto set = [&](std::size_t index, double value) {
    if constexpr (std::is_void_v<decltype(interfaces[index].set_value(
                      value))>) {
      interfaces[index].set_value(value);
      return true;
    } else {
      return static_cast<bool>(interfaces[index].set_value(value));
    }
  };
  bool ok = true;
  for (std::size_t i = 0; i < 4; ++i) {
    ok = set(indices[i], targets.steering[i]) && ok;
    ok = set(indices[i + 4], targets.wheels[i]) && ok;
  }
  return ok;
}
} // namespace swerve_gazebo_sim
