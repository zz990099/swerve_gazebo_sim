// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "rclcpp/rclcpp.hpp"
namespace swerve_gazebo_sim {
// Complete periodically refreshed snapshots: prefer the newest observation to
// retransmitting obsolete samples. All source stamps and age gates remain intact.
inline rclcpp::QoS snapshot_qos() {
  return rclcpp::QoS(1).best_effort().durability_volatile();
}
} // namespace swerve_gazebo_sim
