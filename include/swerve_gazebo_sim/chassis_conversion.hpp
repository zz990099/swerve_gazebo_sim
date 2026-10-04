// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "swerve_gazebo_sim/chassis_runtime.hpp"
#include "swerve_gazebo_sim/msg/chassis_command.hpp"
#include "swerve_gazebo_sim/msg/chassis_context.hpp"
#include "swerve_gazebo_sim/msg/chassis_state.hpp"

namespace swerve_gazebo_sim {
inline double seconds(const builtin_interfaces::msg::Time &t) {
  if (t.sec < 0 || t.nanosec >= 1000000000)
    return -1;
  return double(t.sec) + double(t.nanosec) * 1e-9;
}
inline builtin_interfaces::msg::Time stamp(double s) {
  builtin_interfaces::msg::Time t;
  if (!std::isfinite(s) || s < 0 || s >= 2147483647.0)
    return t;
  const auto ns = static_cast<std::int64_t>(std::llround(s * 1e9));
  t.sec = static_cast<std::int32_t>(ns / 1000000000);
  t.nanosec = static_cast<std::uint32_t>(ns % 1000000000);
  return t;
}
inline core::Twist2d twist(const geometry_msgs::msg::Vector3 &v) {
  return {v.x, v.y, v.z};
}
inline geometry_msgs::msg::Vector3 vector(const core::Twist2d &v) {
  geometry_msgs::msg::Vector3 out;
  out.x = v.vx;
  out.y = v.vy;
  out.z = v.wz;
  return out;
}
inline bool finite(const geometry_msgs::msg::Vector3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
inline ChassisPacket decode(const msg::ChassisCommand &m,
                            const std::string &body_frame) {
  ChassisPacket p;
  p.arm = m.arm;
  auto &e = p.envelope;
  e.session_id = m.session_id;
  e.sequence = m.sequence;
  e.issued_at_s = seconds(m.header.stamp);
  e.source_stamp_s = seconds(m.source_stamp);
  e.execute_at_s = seconds(m.execute_at);
  e.valid_until_s = seconds(m.valid_until);
  p.malformed =
      m.header.frame_id != body_frame || m.mode > 2 || m.heading_policy > 1 ||
      m.source_path.empty() || m.source_path.size() > 4096 ||
      !finite(m.velocity) || !finite(m.entry_velocity) ||
      (m.has_mode_request && (m.request_id == 0 || m.requested_mode > 2)) ||
      e.issued_at_s < 0 || e.source_stamp_s < 0 || e.execute_at_s < 0 ||
      e.valid_until_s < 0 || e.source_stamp_s > e.issued_at_s ||
      e.issued_at_s > e.execute_at_s || e.execute_at_s > e.valid_until_s ||
      e.valid_until_s - e.source_stamp_s > .15;
  if (m.authorized) {
    core::ChassisCommand c;
    c.mode = static_cast<core::DriveMode>(m.mode);
    c.target_velocity = twist(m.velocity);
    if (m.has_mode_request)
      c.mode_request = core::ModeRequest{
          m.request_id, static_cast<core::DriveMode>(m.requested_mode),
          twist(m.entry_velocity)};
    e.command.command = c;
  }
  core::CommandTask task;
  task.path_id = m.path_id;
  task.heading_policy = static_cast<core::PathHeadingPolicy>(m.heading_policy);
  for (const auto &pose : m.source_path) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.theta))
      p.malformed = true;
    task.reference_path.push_back({pose.x, pose.y, pose.theta});
  }
  e.source_task = std::move(task);
  return p;
}
inline ChassisContext decode(const msg::ChassisContext &m,
                             const std::string &odom_frame) {
  ChassisContext c;
  c.stamp_s = seconds(m.header.stamp);
  c.valid = m.header.frame_id == odom_frame && c.stamp_s >= 0 &&
            m.heading_policy <= 1 && !m.path.empty() && m.path.size() <= 4096 &&
            m.obstacles.size() <= 128;
  c.input.path_id = m.path_id;
  c.input.heading_policy =
      static_cast<core::PathHeadingPolicy>(m.heading_policy);
  for (const auto &p : m.path) {
    c.valid = c.valid && std::isfinite(p.x) && std::isfinite(p.y) &&
              std::isfinite(p.theta);
    c.input.reference_path.push_back({p.x, p.y, p.theta});
  }
  for (const auto &o : m.obstacles) {
    c.valid = c.valid && std::isfinite(o.x) && std::isfinite(o.y) &&
              std::isfinite(o.radius) && o.radius >= 0;
    c.input.obstacles.push_back({o.x, o.y, o.radius});
  }
  return c;
}
inline msg::ChassisState encode(const ChassisRuntime &r,
                                const std::string &odom,
                                const std::string &body) {
  msg::ChassisState m;
  const auto &s = r.state();
  m.header.stamp = stamp(s.stamp_s);
  m.header.frame_id = odom;
  m.body_frame = body;
  m.pose.x = s.pose.x;
  m.pose.y = s.pose.y;
  m.pose.theta = s.pose.yaw;
  m.velocity = vector(s.velocity);
  m.steering_angles = s.steering_angles;
  m.wheel_speeds = s.wheel_speeds;
  m.actual_mode = static_cast<std::uint8_t>(s.actual_mode);
  m.confirmed = s.mode_confirmed && !r.fault();
  m.fault = r.fault();
  m.phase = static_cast<std::uint8_t>(r.fault() ? core::TransitionPhase::Fault
                                                : r.phase());
  m.request_id = s.mode_request_id;
  m.time_in_mode = s.time_in_mode_s;
  m.has_accepted_request = s.accepted_mode_request.has_value();
  if (s.accepted_mode_request) {
    const auto &a = *s.accepted_mode_request;
    m.accepted_mode = static_cast<std::uint8_t>(a.mode);
    m.accepted_entry_velocity = vector(a.entry_velocity);
    m.accepted_steering = a.steering_targets;
  }
  m.session_id = r.session();
  m.sequence = r.sequence();
  m.endpoint_fault = static_cast<std::uint8_t>(r.reason());
  m.timing_error = static_cast<std::uint8_t>(r.timing_error());
  m.safety_error = static_cast<std::uint8_t>(r.safety_error());
  m.next_execute_at = stamp(r.next_tick());
  return m;
}
} // namespace swerve_gazebo_sim
