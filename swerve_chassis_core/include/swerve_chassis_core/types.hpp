// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstdint>
namespace swerve_chassis_core
{
constexpr double pi = 3.14159265358979323846;
using Wheels = std::array<double, 4>;
struct Twist
{
  double vx{}, vy{}, wz{};
};
struct Joints
{
  Wheels steering{}, speed{};
};  // rad, rad/s; FL FR RL RR
struct Geometry
{
  double wheelbase{0.6}, track_width{0.5}, radius{0.1};
};
struct Limits
{
  double steering{pi / 2}, steering_rate{2.5}, wheel_speed{20}, wheel_acceleration{40};
  double linear_speed{0.8}, angular_speed{0.8}, linear_acceleration{0.9}, angular_acceleration{1.3};
  double tracking_tolerance{0.05}, stopped_speed{0.05};
};
enum class Reason : uint8_t { tracking, limited, braking, presteering, stopped };
// Complete prediction snapshot: no hidden mutable history in Generator.
struct State
{
  Joints command{};
  bool valid{false};
};
struct Result
{
  State state;
  Twist velocity;
  Reason reason;
  double residual;
};
struct Pose
{
  double x{}, y{}, yaw{};
};
void validate(const Geometry &, const Limits &);
bool finite(const Twist &);
bool finite(const Joints &);
}  // namespace swerve_chassis_core
