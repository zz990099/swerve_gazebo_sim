// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace swerve_gazebo_sim {
constexpr double pi = 3.14159265358979323846;
struct Twist {
  double vx = 0, vy = 0, wz = 0;
};
struct JointTargets {
  std::array<double, 4> steering{}, wheels{};
}; // rad, rad/s
// Explicit, algorithm-independent chassis modes and mechanical phases.
enum class Mode : std::uint8_t { DualAckermann, Spin, Crab };
enum class Phase : std::uint8_t { Fault, Braking, Aligning, Ready };
enum class Fault : std::uint8_t {
  None,
  Disarmed,
  Clock,
  Feedback,
  Command,
  Timeout,
  Transition,
  Write
};
struct Config {
  double wheelbase = .6, track_width = .5, wheel_radius = .1,
         steering_limit = pi / 2;
  double max_wheel_speed = 20, max_wheel_acceleration = 40,
         max_steering_rate = 2.5;
  double steering_alignment_tolerance = .05, steering_alignment_duration = .05;
  double mode_switch_timeout = 5, stopped_wheel_speed = .05, cmd_timeout = .5,
         wall_timeout = .5;
  double update_rate = 100, drive_steering_limit = .2, max_linear_speed = .8,
         max_angular_speed = .8;
  double max_linear_acceleration = .9, max_angular_acceleration = 1.3;
};
struct Command {
  std::uint64_t session = 0, sequence = 0, request = 0, receipt = 0;
  double stamp = -1, wall = -1;
  Mode mode = Mode::DualAckermann;
  Twist velocity, entry;
  bool reset = false, valid = true;
};
inline bool finite(Twist t) {
  return std::isfinite(t.vx) && std::isfinite(t.vy) && std::isfinite(t.wz);
}
inline bool zero(Twist t) { return t.vx == 0 && t.vy == 0 && t.wz == 0; }
inline bool equal(Twist a, Twist b) {
  return a.vx == b.vx && a.vy == b.vy && a.wz == b.wz;
}
inline bool permitted(Mode m, Twist t) {
  if (!finite(t))
    return false;
  switch (m) {
  case Mode::DualAckermann:
    return t.vy == 0 && (t.vx != 0 || t.wz == 0);
  case Mode::Spin:
    return t.vx == 0 && t.vy == 0;
  case Mode::Crab:
    return t.wz == 0;
  }
  return false;
}
inline void validate(const Config &c) {
  const double values[] = {c.wheelbase,
                           c.track_width,
                           c.wheel_radius,
                           c.steering_limit,
                           c.max_wheel_speed,
                           c.max_wheel_acceleration,
                           c.max_steering_rate,
                           c.steering_alignment_tolerance,
                           c.steering_alignment_duration,
                           c.mode_switch_timeout,
                           c.stopped_wheel_speed,
                           c.cmd_timeout,
                           c.wall_timeout,
                           c.update_rate,
                           c.drive_steering_limit,
                           c.max_linear_speed,
                           c.max_angular_speed,
                           c.max_linear_acceleration,
                           c.max_angular_acceleration};
  for (double v : values)
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument(
          "Positive finite chassis parameters required");
  if (std::abs(c.steering_limit - pi / 2) > 1e-9 || c.update_rate != 100 ||
      c.mode_switch_timeout <= c.steering_alignment_duration ||
      c.drive_steering_limit < c.steering_alignment_tolerance)
    throw std::invalid_argument("Invalid chassis timing or steering settings");
}
class ChassisRuntime {
public:
  explicit ChassisRuntime(Config c = {}) : c_(c) { validate(c_); }
  Phase phase() const { return phase_; }
  Fault fault() const { return fault_; }
  Mode actual_mode() const { return actual_; }
  Mode requested_mode() const { return requested_; }
  bool confirmed() const { return phase_ == Phase::Ready; }
  std::uint64_t session() const { return session_; }
  std::uint64_t sequence() const { return sequence_; }
  std::uint64_t request() const { return request_; }
  Twist entry() const { return entry_; }
  const std::array<double, 4> &entry_angles() const { return accepted_angles_; }
  void stop(Fault f = Fault::Disarmed) {
    fault_ = f;
    phase_ = Phase::Fault;
    velocity_ = {};
    limited_ = {};
  }
  JointTargets stopped_targets(const JointTargets &s) const {
    JointTargets out;
    for (std::size_t i = 0; i < 4; ++i)
      out.steering[i] =
          std::isfinite(s.steering[i])
              ? std::clamp(s.steering[i], -c_.steering_limit, c_.steering_limit)
              : 0;
    return out;
  }
  Twist forward(const JointTargets &s) const {
    Twist out;
    for (std::size_t i = 0; i < 4; ++i) {
      const double x = (i < 2 ? 1 : -1) * c_.wheelbase / 2,
                   y = (i % 2 == 0 ? 1 : -1) * c_.track_width / 2;
      const double vx = s.wheels[i] * c_.wheel_radius * std::cos(s.steering[i]),
                   vy = s.wheels[i] * c_.wheel_radius * std::sin(s.steering[i]);
      out.vx += vx / 4;
      out.vy += vy / 4;
      out.wz += (-y * vx + x * vy) /
                (c_.wheelbase * c_.wheelbase + c_.track_width * c_.track_width);
    }
    return out;
  }
  JointTargets inverse(Twist t, const std::array<double, 4> &current) const {
    JointTargets out;
    out.steering = current;
    for (std::size_t i = 0; i < 4; ++i) {
      const double x = (i < 2 ? 1 : -1) * c_.wheelbase / 2,
                   y = (i % 2 == 0 ? 1 : -1) * c_.track_width / 2;
      const double vx = t.vx - t.wz * y, vy = t.vy + t.wz * x,
                   speed = std::hypot(vx, vy) / c_.wheel_radius;
      if (speed < 1e-12)
        continue;
      double best = 1e100;
      const double raw = std::atan2(vy, vx);
      for (int k = -1; k <= 1; ++k) {
        const double a = raw + k * pi;
        if (a < -c_.steering_limit - 1e-12 || a > c_.steering_limit + 1e-12)
          continue;
        if (std::abs(a - current[i]) < best) {
          best = std::abs(a - current[i]);
          out.steering[i] =
              std::clamp(a, -c_.steering_limit, c_.steering_limit);
          out.wheels[i] = k == 0 ? speed : -speed;
        }
      }
    }
    double scale = 1;
    for (double w : out.wheels)
      scale = std::max(scale, std::abs(w) / c_.max_wheel_speed);
    for (double &w : out.wheels)
      w /= scale;
    return out;
  }
  JointTargets update(double now, double wall, const JointTargets &measured,
                      const Command *packet = nullptr) {
    const bool clock = std::isfinite(now) && now >= 0 && std::isfinite(wall) &&
                       wall >= 0 && (last_now_ < 0 || now >= last_now_) &&
                       (last_wall_ < 0 || wall >= last_wall_);
    const double dt = clock && last_now_ >= 0 ? now - last_now_ : 0;
    const bool gap = last_wall_ >= 0 && wall - last_wall_ > c_.wall_timeout;
    last_now_ = std::isfinite(now) && now >= 0 ? now : -1;
    last_wall_ = std::isfinite(wall) && wall >= 0 ? wall : -1;
    if (!clock || dt > .03 || gap) {
      stop(Fault::Clock);
      return stopped_targets(measured);
    }
    for (std::size_t i = 0; i < 4; ++i)
      if (!std::isfinite(measured.steering[i]) ||
          !std::isfinite(measured.wheels[i]) ||
          std::abs(measured.steering[i]) > c_.steering_limit + .05) {
        stop(Fault::Feedback);
        return stopped_targets(measured);
      }
    if (!initialized_) {
      sent_ = stopped_targets(measured);
      initialized_ = true;
    }
    if (phase_ != Phase::Fault && (now - stamp_ > c_.cmd_timeout ||
                                   wall - receipt_wall_ > c_.wall_timeout))
      stop(Fault::Timeout);
    if (packet && packet->receipt != receipt_) {
      receipt_ = packet->receipt;
      const auto &p = *packet;
      if (!p.valid || !p.session || !p.sequence || !std::isfinite(p.stamp) ||
          p.stamp < 0 || p.stamp > now + 1e-9 ||
          now - p.stamp > c_.cmd_timeout || !std::isfinite(p.wall) ||
          p.wall > wall || wall - p.wall > c_.wall_timeout ||
          !permitted(p.mode, p.velocity) || !permitted(p.mode, p.entry))
        stop(Fault::Command);
      else if (p.reset) {
        if (p.session <= session_ || !zero(p.velocity) || !zero(p.entry) ||
            p.request != 0 || p.mode != actual_ || !stopped(measured))
          stop(Fault::Command);
        else {
          session_ = p.session;
          sequence_ = p.sequence;
          request_ = 0;
          requested_ = actual_;
          entry_ = {};
          velocity_ = {};
          limited_ = {};
          sent_ = stopped_targets(measured);
          alignment_ = measured.steering;
          accepted_angles_ = alignment_;
          explicit_transition_ = false;
          phase_ = Phase::Ready;
          fault_ = Fault::None;
          stamp_ = p.stamp;
          receipt_wall_ = p.wall;
        }
      } else if (phase_ != Phase::Fault) {
        if (p.session != session_ || p.sequence <= sequence_ ||
            p.stamp < stamp_ || p.request < request_)
          stop(Fault::Command);
        else if (p.request > request_) {
          if (phase_ != Phase::Ready || !zero(p.velocity))
            stop(Fault::Command);
          else {
            request_ = p.request;
            requested_ = p.mode;
            entry_ = p.entry;
            Twist intent = bounded(entry_);
            if (zero(intent))
              intent =
                  requested_ == Mode::Spin ? Twist{0, 0, 1} : Twist{1, 0, 0};
            alignment_ = inverse(intent, measured.steering).steering;
            accepted_angles_ = alignment_;
            explicit_transition_ = true;
            begin(now);
            velocity_ = {};
          }
        } else if (p.mode != requested_ || !equal(p.entry, entry_) ||
                   (explicit_transition_ && !confirmed() && !zero(p.velocity)))
          stop(Fault::Command);
        else
          velocity_ = bounded(p.velocity);
        if (phase_ != Phase::Fault) {
          sequence_ = p.sequence;
          stamp_ = p.stamp;
          receipt_wall_ = p.wall;
        }
      }
    }
    if (phase_ == Phase::Fault) {
      sent_ = stopped_targets(measured);
      return sent_;
    }
    if (dt <= 0)
      return sent_;
    if (phase_ == Phase::Ready) {
      if (zero(velocity_)) {
        ramp_wheels({}, dt);
        limited_ = {};
        return sent_;
      }
      const Twist next = slew(velocity_, dt);
      const auto target = inverse(next, measured.steering);
      if (error(target.steering, measured.steering) > c_.drive_steering_limit) {
        alignment_ = inverse(velocity_, measured.steering).steering;
        begin(now);
      } else {
        ramp_wheels(target.wheels, dt);
        ramp_steering(target.steering, dt);
        limited_ = next;
        return sent_;
      }
    }
    if (now - transition_start_ > c_.mode_switch_timeout) {
      stop(Fault::Transition);
      sent_ = stopped_targets(measured);
      return sent_;
    }
    if (phase_ == Phase::Braking) {
      ramp_wheels({}, dt);
      limited_ = {};
      if (!stopped(measured) || max_abs(sent_.wheels) > 1e-12)
        return sent_;
      phase_ = Phase::Aligning;
    }
    if (!stopped(measured)) {
      phase_ = Phase::Braking;
      aligned_since_ = -1;
      return sent_;
    }
    ramp_steering(alignment_, dt);
    if (error(alignment_, measured.steering) <=
        c_.steering_alignment_tolerance) {
      if (aligned_since_ < 0)
        aligned_since_ = now;
      if (now - aligned_since_ + 1e-9 >= c_.steering_alignment_duration) {
        actual_ = requested_;
        phase_ = Phase::Ready;
        explicit_transition_ = false;
      }
    } else
      aligned_since_ = -1;
    return sent_;
  }

private:
  static double max_abs(const std::array<double, 4> &a) {
    double v = 0;
    for (double x : a)
      v = std::max(v, std::abs(x));
    return v;
  }
  static double error(const std::array<double, 4> &a,
                      const std::array<double, 4> &b) {
    double v = 0;
    for (std::size_t i = 0; i < 4; ++i)
      v = std::max(v, std::abs(a[i] - b[i]));
    return v;
  }
  bool stopped(const JointTargets &s) const {
    return max_abs(s.wheels) <= c_.stopped_wheel_speed;
  }
  Twist bounded(Twist t) const {
    const double scale = std::max({1., std::abs(t.vx) / c_.max_linear_speed,
                                   std::abs(t.vy) / c_.max_linear_speed,
                                   std::abs(t.wz) / c_.max_angular_speed});
    t = {t.vx / scale, t.vy / scale, t.wz / scale};
    const double d = std::hypot(t.vx, t.vy);
    if (d > c_.max_linear_speed) {
      t.vx *= c_.max_linear_speed / d;
      t.vy *= c_.max_linear_speed / d;
    }
    return t;
  }
  Twist slew(Twist t, double dt) const {
    const double dx = t.vx - limited_.vx, dy = t.vy - limited_.vy,
                 dw = t.wz - limited_.wz;
    const double linear = std::hypot(dx, dy), angular = std::abs(dw);
    double scale = 1;
    if (linear > 0)
      scale = std::min(scale, c_.max_linear_acceleration * dt / linear);
    if (angular > 0)
      scale = std::min(scale, c_.max_angular_acceleration * dt / angular);
    return {limited_.vx + scale * dx, limited_.vy + scale * dy,
            limited_.wz + scale * dw};
  }
  void begin(double now) {
    phase_ = Phase::Braking;
    transition_start_ = now;
    aligned_since_ = -1;
  }
  void ramp_wheels(const std::array<double, 4> &t, double dt) {
    for (std::size_t i = 0; i < 4; ++i)
      sent_.wheels[i] +=
          std::clamp(t[i] - sent_.wheels[i], -c_.max_wheel_acceleration * dt,
                     c_.max_wheel_acceleration * dt);
  }
  void ramp_steering(const std::array<double, 4> &t, double dt) {
    for (std::size_t i = 0; i < 4; ++i)
      sent_.steering[i] +=
          std::clamp(t[i] - sent_.steering[i], -c_.max_steering_rate * dt,
                     c_.max_steering_rate * dt);
  }
  Config c_;
  JointTargets sent_;
  bool initialized_ = false, explicit_transition_ = false;
  Phase phase_ = Phase::Fault;
  Fault fault_ = Fault::Disarmed;
  Mode actual_ = Mode::DualAckermann, requested_ = Mode::DualAckermann;
  Twist velocity_, entry_, limited_;
  std::array<double, 4> alignment_{}, accepted_angles_{};
  std::uint64_t session_ = 0, sequence_ = 0, request_ = 0, receipt_ = 0;
  double last_now_ = -1, last_wall_ = -1, stamp_ = -1, receipt_wall_ = -1,
         transition_start_ = 0, aligned_since_ = -1;
};
} // namespace swerve_gazebo_sim
