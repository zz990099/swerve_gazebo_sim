// SPDX-License-Identifier: Apache-2.0
#include "swerve_chassis_core/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace swerve_chassis_core
{
bool finite(const Twist & t)
{
  return std::isfinite(t.vx) && std::isfinite(t.vy) && std::isfinite(t.wz);
}
bool finite(const Joints & j)
{
  for (size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(j.steering[i]) || !std::isfinite(j.speed[i])) {
      return false;
    }
  }
  return true;
}
void validate(const Geometry & g, const Limits & l)
{
  for (double v : {g.wheelbase, g.track_width, g.radius, l.steering, l.steering_rate, l.wheel_speed,
                   l.wheel_acceleration, l.linear_speed, l.angular_speed, l.linear_acceleration,
                   l.angular_acceleration, l.tracking_tolerance, l.stopped_speed}) {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("Limits must be finite and positive");
    }
  }
  if (std::abs(l.steering - pi / 2) > 1e-9) {
    throw std::invalid_argument("Requires +/- pi/2 steering");
  }
}
Twist scale(Twist t, double s)
{
  return {t.vx * s, t.vy * s, t.wz * s};
}
Joints inverse(const Geometry & g, Twist t, const Wheels & reference)
{
  Joints out;
  for (size_t i = 0; i < 4; ++i) {
    const double x = (i < 2 ? 1 : -1) * g.wheelbase / 2;
    const double y = (i % 2 == 0 ? 1 : -1) * g.track_width / 2;
    const double u = t.vx - t.wz * y, v = t.vy + t.wz * x;
    double a = std::atan2(v, u), speed = std::hypot(u, v) / g.radius;
    if (speed < 1e-10) {
      out.steering[i] = reference[i];
      continue;
    }
    if (a > pi / 2) {
      a -= pi;
      speed = -speed;
    }
    if (a < -pi / 2) {
      a += pi;
      speed = -speed;
    }
    if (std::abs(std::abs(a) - pi / 2) < 1e-12 &&
        std::abs(-a - reference[i]) < std::abs(a - reference[i])) {
      a = -a;
      speed = -speed;
    }
    out.steering[i] = a;
    out.speed[i] = speed;
  }
  return out;
}
Twist forward(const Geometry & g, const Joints & j)
{
  Twist t;
  for (size_t i = 0; i < 4; ++i) {
    double u = g.radius * j.speed[i] * std::cos(j.steering[i]);
    double v = g.radius * j.speed[i] * std::sin(j.steering[i]);
    double x = (i < 2 ? 1 : -1) * g.wheelbase / 2;
    double y = (i % 2 == 0 ? 1 : -1) * g.track_width / 2;
    t.vx += u / 4;
    t.vy += v / 4;
    t.wz += (x * v - y * u) / (g.wheelbase * g.wheelbase + g.track_width * g.track_width);
  }
  return t;
}
double residual(const Geometry & g, const Joints & j)
{
  const auto t = forward(g, j);
  double sum = 0;
  for (size_t i = 0; i < 4; ++i) {
    double x = (i < 2 ? 1 : -1) * g.wheelbase / 2;
    double y = (i % 2 == 0 ? 1 : -1) * g.track_width / 2;
    double u = g.radius * j.speed[i] * std::cos(j.steering[i]) - (t.vx - t.wz * y);
    double v = g.radius * j.speed[i] * std::sin(j.steering[i]) - (t.vy + t.wz * x);
    sum += u * u + v * v;
  }
  return std::sqrt(sum / 4);
}
Pose integrate(Pose p, Twist t, double dt)
{
  double a = t.wz * dt;
  double s = std::abs(a) < 1e-8 ? 1 - a * a / 6 : std::sin(a) / a;
  double c = std::abs(a) < 1e-8 ? a / 2 : (1 - std::cos(a)) / a;
  double x = dt * (s * t.vx - c * t.vy), y = dt * (c * t.vx + s * t.vy);
  p.x += std::cos(p.yaw) * x - std::sin(p.yaw) * y;
  p.y += std::sin(p.yaw) * x + std::cos(p.yaw) * y;
  p.yaw = std::remainder(p.yaw + a, 2 * pi);
  return p;
}
Joints response(const Joints & actual, const Joints & target, const Limits & l, double dt)
{
  if (!finite(actual) || !finite(target) || !std::isfinite(dt) || dt <= 0) {
    throw std::invalid_argument("Invalid response input");
  }
  Joints out = actual;
  for (size_t i = 0; i < 4; ++i) {
    out.steering[i] += std::clamp(target.steering[i] - actual.steering[i], -l.steering_rate * dt,
                                  l.steering_rate * dt);
    out.speed[i] += std::clamp(target.speed[i] - actual.speed[i], -l.wheel_acceleration * dt,
                               l.wheel_acceleration * dt);
  }
  return out;
}
}  // namespace swerve_chassis_core
