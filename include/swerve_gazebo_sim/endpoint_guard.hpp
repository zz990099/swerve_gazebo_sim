// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace swerve_gazebo_sim {
// Wire format v1, Float64MultiArray.data[14]:
// session, sequence, arm(0/1), sample_sim_s, valid_until_sim_s,
// steering[FL,FR,RL,RR] rad, wheels[FL,FR,RL,RR] rad/s, version(1).
// IDs are exact positive integers below 2^53. Transport is not authentication.
struct EndpointPacket {
  std::array<double, 14> data{};
  double received_wall_s = -1;
  std::uint64_t receipt = 0;
  bool malformed = false;
};
struct EndpointTargets {
  std::array<double, 4> steering{};
  std::array<double, 4> wheels{};
};
struct EndpointLimits {
  double wall_timeout_s = .1;
  double max_validity_s = .03;
  double max_wheel_speed_radps = 20;
  double steering_limit_rad = 1.5707963267948966;
  double stopped_wheel_radps = .05;
  double arm_steering_tolerance_rad = .05;
};
// Only the controller update thread owns this state. Faults latch, stop all
// wheels and hold measured steering. Expiry is checked BEFORE new admission.
class EndpointGuard {
public:
  explicit EndpointGuard(EndpointLimits limits = {}) : limits_(limits) {
    for (double v :
         {limits.wall_timeout_s, limits.max_validity_s, limits.max_wheel_speed_radps,
          limits.steering_limit_rad, limits.stopped_wheel_radps, limits.arm_steering_tolerance_rad})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument("endpoint limits must be finite and positive");
  }
  void deactivate() { fault_ = true; } // Session high-water mark survives lifecycle changes.
  bool fault() const { return fault_; }
  std::uint64_t session() const { return static_cast<std::uint64_t>(session_); }
  std::uint64_t sequence() const { return static_cast<std::uint64_t>(sequence_); }
  EndpointTargets update(double sim_s, double wall_s, const EndpointTargets &measured,
                         const std::optional<EndpointPacket> &packet = std::nullopt) {
    bool clocks_ok = std::isfinite(sim_s) && sim_s >= 0 && std::isfinite(wall_s) && wall_s >= 0 &&
                     sim_s >= last_sim_s_ && wall_s >= last_wall_s_;
    bool feedback_ok = true;
    for (std::size_t i = 0; i < 4; ++i)
      feedback_ok = feedback_ok && std::isfinite(measured.wheels[i]) &&
                    std::isfinite(measured.steering[i]) &&
                    std::abs(measured.steering[i]) <= limits_.steering_limit_rad + 1e-9;
    const bool discontinuity = !clocks_ok;
    if (!clocks_ok || !feedback_ok ||
        (!fault_ &&
         (sim_s > valid_until_s_ + 1e-9 || wall_s - received_wall_s_ >= limits_.wall_timeout_s)))
      fault_ = true;
    // A clock reset requires a later healthy stopped arm tick, never a packet
    // that conceals the reset on the same update.
    if (std::isfinite(sim_s) && sim_s >= 0)
      last_sim_s_ = sim_s;
    if (std::isfinite(wall_s) && wall_s >= 0)
      last_wall_s_ = wall_s;
    if (packet && packet->receipt != receipt_) {
      receipt_ = packet->receipt;
      const auto &d = packet->data;
      bool valid = !packet->malformed && clocks_ok && feedback_ok &&
                   std::isfinite(packet->received_wall_s) && packet->received_wall_s <= wall_s &&
                   wall_s - packet->received_wall_s < limits_.wall_timeout_s;
      for (double v : d)
        valid = valid && std::isfinite(v);
      auto id_ok = [](double v) { return v >= 1 && v < 9007199254740992.0 && std::floor(v) == v; };
      valid = valid && id_ok(d[0]) && id_ok(d[1]) && (d[2] == 0 || d[2] == 1) && d[13] == 1 &&
              d[3] >= 0 && d[3] <= sim_s + 1e-9 && d[4] > d[3] &&
              d[4] - d[3] <= limits_.max_validity_s + 1e-9 && sim_s <= d[4] + 1e-9;
      for (std::size_t i = 0; i < 4; ++i)
        valid = valid && std::abs(d[5 + i]) <= limits_.steering_limit_rad + 1e-9 &&
                std::abs(d[9 + i]) <= limits_.max_wheel_speed_radps + 1e-9;
      if (valid && d[2] == 1) {
        valid = fault_ && !discontinuity && d[0] > session_;
        for (std::size_t i = 0; i < 4; ++i)
          valid = valid && d[9 + i] == 0 &&
                  std::abs(measured.wheels[i]) <= limits_.stopped_wheel_radps &&
                  std::abs(d[5 + i] - measured.steering[i]) <= limits_.arm_steering_tolerance_rad;
        if (valid) {
          session_ = d[0];
          sequence_ = 0;
          stamp_s_ = -1;
          fault_ = false;
        }
      } else if (valid) {
        valid = !fault_ && d[0] == session_;
      }
      valid = valid && d[1] > sequence_ && d[3] >= stamp_s_;
      if (!valid)
        fault_ = true;
      else {
        sequence_ = d[1];
        stamp_s_ = d[3];
        valid_until_s_ = d[4];
        received_wall_s_ = packet->received_wall_s;
        for (std::size_t i = 0; i < 4; ++i) {
          targets_.steering[i] = d[5 + i];
          targets_.wheels[i] = d[9 + i];
        }
      }
    }
    if (fault_) {
      targets_.wheels.fill(0);
      // Never propagate NaN to a command interface. Hold the last finite angle
      // if feedback is corrupt; use bounded measured angles otherwise.
      for (std::size_t i = 0; i < 4; ++i)
        if (std::isfinite(measured.steering[i]))
          targets_.steering[i] = std::clamp(measured.steering[i], -limits_.steering_limit_rad,
                                            limits_.steering_limit_rad);
    }
    return targets_;
  }

private:
  EndpointLimits limits_;
  EndpointTargets targets_;
  bool fault_ = true;
  double session_ = 0, sequence_ = 0, stamp_s_ = -1, valid_until_s_ = -1;
  double received_wall_s_ = -1, last_sim_s_ = -1, last_wall_s_ = -1;
  std::uint64_t receipt_ = 0;
};
} // namespace swerve_gazebo_sim
