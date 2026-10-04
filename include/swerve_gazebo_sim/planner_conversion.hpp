// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "swerve_gazebo_sim/chassis_conversion.hpp"
#include "swerve_gazebo_sim/planner_bridge.hpp"

namespace swerve_gazebo_sim {
inline PlannerFeedback decode(const msg::ChassisState &m) {
  PlannerFeedback f;
  auto &s = f.vehicle;
  s.pose = {m.pose.x, m.pose.y, m.pose.theta};
  s.velocity = twist(m.velocity);
  s.steering_angles = m.steering_angles;
  s.wheel_speeds = m.wheel_speeds; // Already linear m/s.
  s.actual_mode = static_cast<core::DriveMode>(m.actual_mode);
  s.mode_confirmed = m.confirmed;
  s.mode_fault = m.fault;
  s.mode_request_id = m.request_id;
  s.time_in_mode_s = m.time_in_mode;
  s.stamp_s = seconds(m.header.stamp);
  if (m.has_accepted_request)
    s.accepted_mode_request = core::JointModeRequest{
        m.request_id, static_cast<core::DriveMode>(m.accepted_mode),
        m.accepted_steering, twist(m.accepted_entry_velocity)};
  f.fault = m.fault;
  f.session = m.session_id;
  f.next_execute_at = seconds(m.next_execute_at);
  return f;
}
inline msg::ChassisCommand encode(const ChassisPacket &p,
                                  const std::string &body) {
  const auto &e = p.envelope;
  msg::ChassisCommand m;
  m.header.frame_id = body;
  m.header.stamp = stamp(e.issued_at_s);
  m.source_stamp = stamp(e.source_stamp_s);
  m.execute_at = stamp(e.execute_at_s);
  m.valid_until = stamp(e.valid_until_s);
  m.session_id = e.session_id;
  m.sequence = e.sequence;
  m.arm = p.arm;
  m.authorized = e.command.command.has_value();
  if (e.command.command) {
    const auto &c = *e.command.command;
    m.mode = static_cast<std::uint8_t>(c.mode);
    m.velocity = vector(c.target_velocity);
    if (c.mode_request) {
      m.has_mode_request = true;
      m.request_id = c.mode_request->id;
      m.requested_mode = static_cast<std::uint8_t>(c.mode_request->mode);
      m.entry_velocity = vector(c.mode_request->entry_velocity);
    }
  }
  if (e.source_task) {
    m.path_id = e.source_task->path_id;
    m.heading_policy = static_cast<std::uint8_t>(e.source_task->heading_policy);
    for (const auto &p : e.source_task->reference_path) {
      geometry_msgs::msg::Pose2D pose;
      pose.x = p.x;
      pose.y = p.y;
      pose.theta = p.yaw;
      m.source_path.push_back(pose);
    }
  }
  return m;
}
} // namespace swerve_gazebo_sim
