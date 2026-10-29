// SPDX-License-Identifier: Apache-2.0
#include "swerve_chassis_core/setpoint_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace swerve_chassis_core
{
namespace
{
double peak(const Wheels & w)
{
  double p = 0;
  for (double v : w) {
    p = std::max(p, std::abs(v));
  }
  return p;
}
}  // namespace
Generator::Generator(Geometry g, Limits l) : geometry_(g), limits_(l)
{
  validate(g, l);
}
Result Generator::step(Twist desired, const Joints & measured, const State & previous,
                       double dt) const
{
  const auto & l = limits_;
  if (!finite(desired) || !finite(measured) || !std::isfinite(dt) || dt <= 0 || dt > 0.1 ||
      (previous.valid &&
       (!finite(previous.command) || residual(geometry_, previous.command) > 1e-7))) {
    throw std::invalid_argument("Invalid generator input or snapshot");
  }
  State state = previous;
  if (!state.valid) {
    state.command.steering = measured.steering;
    state.command.speed.fill(0);
    state.valid = true;
  }
  for (double a : state.command.steering) {
    if (std::abs(a) > l.steering + 1e-9) {
      throw std::invalid_argument("Snapshot steering out of range");
    }
  }
  if (peak(state.command.speed) > l.wheel_speed + 1e-9) {
    throw std::invalid_argument("Snapshot speed out of range");
  }
  const Joints old = state.command;
  const Twist start = forward(geometry_, old);
  double factor =
      std::min({1.0, l.linear_speed / std::max(1e-12, std::hypot(desired.vx, desired.vy)),
                l.angular_speed / std::max(1e-12, std::abs(desired.wz))});
  desired = scale(desired, factor);
  desired = scale(
      desired,
      std::min(1.0, l.wheel_speed / std::max(1e-12, peak(inverse(geometry_, desired).speed))));
  const bool zero = std::hypot(desired.vx, desired.vy) + std::abs(desired.wz) < 1e-10;
  bool tracking = true;
  for (size_t i = 0; i < 4; ++i) {
    tracking &= std::abs(old.steering[i] - measured.steering[i]) <= l.tracking_tolerance;
    tracking &= std::abs(old.speed[i] - measured.speed[i]) <= l.wheel_acceleration * 0.1;
  }
  auto candidate = [&](double a) {
    return inverse(geometry_,
                   {start.vx + a * (desired.vx - start.vx), start.vy + a * (desired.vy - start.vy),
                    start.wz + a * (desired.wz - start.wz)},
                   old.steering);
  };
  auto feasible = [&](const Joints & j) {
    const auto t = forward(geometry_, j);
    if (std::hypot(t.vx - start.vx, t.vy - start.vy) > l.linear_acceleration * dt + 1e-12 ||
        std::abs(t.wz - start.wz) > l.angular_acceleration * dt + 1e-12) {
      return false;
    }
    for (size_t i = 0; i < 4; ++i) {
      if (std::abs(j.steering[i] - old.steering[i]) > l.steering_rate * dt + 1e-12 ||
          std::abs(j.steering[i] - measured.steering[i]) >
              l.tracking_tolerance + l.steering_rate * dt ||
          std::abs(j.speed[i] - old.speed[i]) > l.wheel_acceleration * dt + 1e-12) {
        return false;
      }
    }
    return true;
  };
  // Follow only the connected feasible prefix. Never jump to a later feasible
  // branch after a steering wrap. Subdivide around individual wheel-vector
  // zeros and x=0 crossings, where the +/-pi/2 representation can change.
  double progress = 0;
  if (tracking && !zero) {
    std::array<double, 14> cuts{};
    size_t count = 0;
    cuts[count++] = 1;
    for (size_t i = 0; i < 4; ++i) {
      const double x = (i < 2 ? 1 : -1) * geometry_.wheelbase / 2;
      const double y = (i % 2 == 0 ? 1 : -1) * geometry_.track_width / 2;
      const double u = start.vx - start.wz * y, v = start.vy + start.wz * x;
      const double du = desired.vx - desired.wz * y - u, dv = desired.vy + desired.wz * x - v;
      const double norm = du * du + dv * dv;
      if (norm > 1e-24) {
        const double closest = -(u * du + v * dv) / norm;
        if (closest > 0 && closest < 1) {
          cuts[count++] = closest;
        }
      }
      for (auto pair : {std::array<double, 2>{u, du}, std::array<double, 2>{v, dv}}) {
        if (std::abs(pair[1]) > 1e-12) {
          double a = -pair[0] / pair[1];
          if (a > 0 && a < 1) {
            cuts[count++] = a;
          }
        }
      }
    }
    std::sort(cuts.begin(), cuts.begin() + count);
    double lower = 0;
    bool blocked = false;
    for (size_t k = 0; k < count && !blocked; ++k) {
      double upper = cuts[k];
      for (int s = 1; s <= 16; ++s) {
        const double a = lower + (upper - lower) * s / 16;
        if (feasible(candidate(a))) {
          progress = a;
          continue;
        }
        double lo = progress, hi = a;
        for (int n = 0; n < 24; ++n) {
          double mid = (lo + hi) / 2;
          if (feasible(candidate(mid))) {
            lo = mid;
          } else {
            hi = mid;
          }
        }
        progress = lo;
        blocked = true;
        break;
      }
      lower = upper;
    }
  }
  Reason reason = Reason::limited;
  if (progress > 1e-6) {
    state.command = candidate(progress);
    reason = progress > 1 - 1e-8 && factor == 1 ? Reason::tracking : Reason::limited;
  } else if (peak(old.speed) > 1e-10) {
    // Common scale preserves all rolling constraints during braking.
    double reduction =
        std::min({1.0, l.wheel_acceleration * dt / peak(old.speed),
                  l.linear_acceleration * dt / std::max(1e-12, std::hypot(start.vx, start.vy)),
                  l.angular_acceleration * dt / std::max(1e-12, std::abs(start.wz))});
    for (double & v : state.command.speed) {
      v *= 1 - reduction;
    }
    reason = Reason::braking;
  } else if (!zero && peak(measured.speed) <= l.stopped_speed) {
    auto target = inverse(geometry_, desired, old.steering);
    for (size_t i = 0; i < 4; ++i) {
      // With both requested and measured drive stopped, reorient continuously
      // at the target-rate limit. Encoder tracking still gates *driving* above;
      // applying that same gate here creates a stop-and-wait steering staircase.
      state.command.steering[i] += std::clamp(target.steering[i] - old.steering[i],
                                            -l.steering_rate * dt, l.steering_rate * dt);
    }
    reason = Reason::presteering;
  } else {
    reason = zero ? Reason::stopped : Reason::braking;
  }
  return {state, forward(geometry_, state.command), reason, residual(geometry_, measured)};
}
}  // namespace swerve_chassis_core
