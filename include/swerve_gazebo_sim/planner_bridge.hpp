// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "swerve_gazebo_sim/chassis_runtime.hpp"
#include "swerve_mppi/planning/controller.hpp"

namespace swerve_gazebo_sim {
enum class PlannerPhase { Disarmed, Arming, Running, Fault };
struct PlannerFeedback {
  core::VehicleState vehicle;
  bool fault = true;
  std::uint64_t session = 0;
  double next_execute_at = -1;
};
// Transport-independent planning owner. Measurements and originating tasks
// retain their timestamps and identity; only execution revalidates at
// application.
class PlannerBridge {
public:
  explicit PlannerBridge(core::Config config = {})
      : config_(config), controller_(config) {}
  PlannerPhase phase() const { return phase_; }
  const core::Output &output() const { return output_; }
  std::uint64_t session() const { return session_; }
  void stop() { phase_ = PlannerPhase::Fault; }
  std::optional<ChassisPacket> arm(const PlannerFeedback &f,
                                   const ChassisContext &c,
                                   std::uint64_t new_session,
                                   double issued_at = -1) {
    if (issued_at == -1)
      issued_at = f.vehicle.stamp_s;
    if (phase_ == PlannerPhase::Running || phase_ == PlannerPhase::Arming ||
        new_session <= std::max(session_, f.session) ||
        new_session >= (std::uint64_t{1} << 53) || !valid(f, c) ||
        !core::is_stopped(f.vehicle, config_) || !std::isfinite(issued_at) ||
        issued_at < f.vehicle.stamp_s || issued_at - f.vehicle.stamp_s > .1)
      return std::nullopt;
    controller_.reset(); // Keeps planner request-ID high water marks.
    session_ = new_session;
    sequence_ = 0;
    last_boundary_ = -1;
    last_source_ = issued_at;
    output_ = {};
    output_.command = core::ChassisCommand{f.vehicle.actual_mode, {}, {}};
    phase_ = PlannerPhase::Arming;
    auto p = packet(f, c, output_, issued_at, true);
    p.envelope.issued_at_s = issued_at;
    return p;
  }
  std::optional<ChassisPacket> plan(const PlannerFeedback &f,
                                    const ChassisContext &c) {
    if (phase_ == PlannerPhase::Arming) {
      // Pre-arm feedback may still be in flight. Never resend the arm packet.
      if (f.session != session_) {
        if (f.session > session_)
          stop();
        return std::nullopt;
      }
      phase_ = f.fault ? PlannerPhase::Fault : PlannerPhase::Running;
    }
    if (phase_ != PlannerPhase::Running)
      return std::nullopt;
    if (f.fault || f.session != session_ || !valid(f, c) ||
        !std::isfinite(f.next_execute_at) ||
        f.next_execute_at <= f.vehicle.stamp_s ||
        f.next_execute_at - f.vehicle.stamp_s > config_.dt_s + 1e-9) {
      stop();
      return std::nullopt;
    }
    if (f.next_execute_at == last_boundary_)
      return std::nullopt;
    if (f.vehicle.stamp_s <= last_source_)
      return std::nullopt;
    if (f.next_execute_at < last_boundary_) {
      stop();
      return std::nullopt;
    }
    auto input = c.input;
    input.vehicle = f.vehicle;
    output_ = controller_.compute(input);
    last_boundary_ = f.next_execute_at;
    last_source_ = f.vehicle.stamp_s;
    auto p = packet(f, c, output_, f.next_execute_at, false);
    if (!output_.command)
      stop();
    return p; // Absent command means an explicit latched cancellation.
  }

private:
  bool valid(const PlannerFeedback &f, const ChassisContext &c) const {
    const auto &s = f.vehicle;
    if (!c.valid || !std::isfinite(c.stamp_s) || !std::isfinite(s.stamp_s) ||
        s.stamp_s < 0 || c.stamp_s > s.stamp_s + 1e-9 ||
        s.stamp_s - c.stamp_s > .15 || c.input.reference_path.empty() ||
        static_cast<unsigned>(c.input.heading_policy) > 1 ||
        c.input.reference_path.size() > config_.max_path_points ||
        c.input.obstacles.size() > config_.max_obstacles ||
        static_cast<unsigned>(s.actual_mode) > 2 ||
        core::check_model_feedback(s, config_).status !=
            core::FeedbackStatus::Valid)
      return false;
    for (const auto &p : c.input.reference_path)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
        return false;
    for (const auto &o : c.input.obstacles)
      if (!std::isfinite(o.x) || !std::isfinite(o.y) ||
          !std::isfinite(o.radius) || o.radius < 0)
        return false;
    return true;
  }
  ChassisPacket packet(const PlannerFeedback &f, const ChassisContext &c,
                       const core::Output &o, double boundary, bool arm) {
    ChassisPacket p;
    p.arm = arm;
    p.envelope = {session_,          ++sequence_,
                  f.vehicle.stamp_s, o,
                  f.vehicle.stamp_s, boundary,
                  boundary + .025,   core::CommandTask::capture(c.input)};
    return p;
  }
  core::Config config_;
  core::Controller controller_;
  core::Output output_;
  PlannerPhase phase_ = PlannerPhase::Disarmed;
  std::uint64_t session_ = 0, sequence_ = 0;
  double last_boundary_ = -1, last_source_ = -1;
};
} // namespace swerve_gazebo_sim
