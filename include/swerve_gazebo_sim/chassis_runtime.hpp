// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "swerve_gazebo_sim/endpoint_guard.hpp"
#include "swerve_mppi/execution/profile_runner.hpp"
#include "swerve_mppi/feedback/feedback.hpp"
#include <memory>

namespace swerve_gazebo_sim {
namespace core = swerve_mppi;
struct ChassisPacket {
  core::CommandEnvelope envelope;
  bool arm = false;
  bool malformed = false;
  double wall_s = -1;
  std::uint64_t receipt = 0;
};
struct ChassisContext {
  core::ControllerInput input;
  double stamp_s = -1;
  bool valid = false;
};
// Simulation execution boundary. Called by the joint owner, never a ROS timer
// pretending that a transported observation is the current physical snapshot.
class ChassisRuntime {
public:
  explicit ChassisRuntime(core::Config config = {})
      : config_(config), kinematics_(config), runner_(config, .15),
        guard_(limits(config)) {}
  void stop(EndpointFault reason = EndpointFault::Disarmed) {
    fault_ = true;
    pending_.reset();
    guard_.deactivate(reason);
    feedback_.fault = true;
    feedback_.confirmed = false;
  }
  bool fault() const { return fault_; }
  EndpointFault reason() const { return guard_.fault_reason(); }
  const core::VehicleState &state() const { return state_; }
  core::TransitionPhase phase() const { return phase_; }
  std::uint64_t session() const { return session_; }
  std::uint64_t sequence() const { return sequence_; }
  double next_tick() const { return next_tick_s_; }
  core::TimingError timing_error() const { return timing_error_; }
  core::ExecutionSafetyError safety_error() const { return safety_error_; }
  EndpointTargets
  update(double now, double wall, const EndpointTargets &joints,
         const ChassisContext &context,
         const std::optional<ChassisPacket> &packet = std::nullopt) {
    const bool clock = std::isfinite(now) && now >= 0 && std::isfinite(wall) &&
                       wall >= 0 && (last_now_ < 0 || now >= last_now_) &&
                       (last_wall_ < 0 || wall >= last_wall_);
    const double elapsed = clock && last_now_ >= 0 ? now - last_now_ : 0;
    if (!clock) {
      stop(EndpointFault::Clock);
      if (std::isfinite(now) && last_now_ >= 0 && now < last_now_)
        state_.pose = {};
    }
    // Encoder odometry, not physical ground truth. Gaps are not extrapolated.
    state_.steering_angles = joints.steering;
    for (std::size_t i = 0; i < 4; ++i)
      state_.wheel_speeds[i] = joints.wheels[i] * config_.wheel_radius_m;
    state_.velocity =
        kinematics_.forward(state_.wheel_speeds, state_.steering_angles);
    state_.stamp_s = now;
    apply_feedback(now);
    const bool valid = core::check_model_feedback(state_, config_).status ==
                       core::FeedbackStatus::Valid;
    if (!valid)
      stop(EndpointFault::Feedback);
    if (valid && elapsed > 0 && elapsed <= .03) {
      const double yaw = state_.pose.yaw + .5 * state_.velocity.wz * elapsed;
      state_.pose.x += (std::cos(yaw) * state_.velocity.vx -
                        std::sin(yaw) * state_.velocity.vy) *
                       elapsed;
      state_.pose.y += (std::sin(yaw) * state_.velocity.vx +
                        std::cos(yaw) * state_.velocity.vy) *
                       elapsed;
      state_.pose.yaw =
          core::wrap_angle(state_.pose.yaw + state_.velocity.wz * elapsed);
    } else if (elapsed > .03) {
      stop(EndpointFault::SimulationDeadline);
    }
    last_now_ = std::isfinite(now) ? now : -1;
    last_wall_ = std::isfinite(wall) ? wall : -1;
    // Check the independent guard BEFORE accepting new commands/recovery.
    auto targets = guard_.update(now, wall, joints);
    if (!fault_ && guard_.fault())
      stop(guard_.fault_reason());
    bool new_arm = false;
    if (packet && packet->receipt != receipt_) {
      receipt_ = packet->receipt;
      const auto &p = *packet;
      if (!clock || !valid || p.malformed || !std::isfinite(p.wall_s) ||
          p.wall_s > wall || wall - p.wall_s >= .15 ||
          !p.envelope.command.command) {
        stop(EndpointFault::CommandRejected);
      } else if (p.arm) {
        if (fault_ && arm(p, now, wall, context)) {
          pending_ = p;
          new_arm = true;
        } else {
          stop(EndpointFault::CommandRejected);
        }
      } else if (!fault_) {
        const auto &e = p.envelope;
        if (e.session_id != session_ || e.sequence <= sequence_ ||
            (pending_ && e.sequence <= pending_->envelope.sequence) ||
            !std::isfinite(e.execute_at_s) ||
            e.execute_at_s > next_tick_s_ + 1e-9 || e.valid_until_s < now ||
            e.issued_at_s > now + 1e-9) {
          stop(EndpointFault::CommandRejected);
        } else {
          pending_ = p;
        }
      }
    }
    if (!fault_ &&
        (!context.valid || context.stamp_s > now + 1e-9 ||
         !std::isfinite(context.stamp_s) || now - context.stamp_s > .15))
      stop(EndpointFault::CommandRejected);
    if (!fault_ && (new_arm || now + 1e-9 >= next_tick_s_)) {
      if (!pending_ || now > next_tick_s_ + 1e-9 ||
          wall - pending_->wall_s >= .15) {
        stop(EndpointFault::SimulationDeadline);
      } else {
        auto latest = context.input;
        latest.vehicle = state_;
        auto result = executor_->update(pending_->envelope, latest, now);
        sequence_ = pending_->envelope.sequence;
        feedback_ = result.execution.feedback;
        feedback_stamp_ = now;
        phase_ = result.execution.phase;
        timing_error_ = result.timing_error;
        safety_error_ = result.safety_error;
        pending_.reset();
        if (!runner_.install(result, now, wall)) {
          stop(EndpointFault::CommandRejected);
        } else {
          next_tick_s_ = now + config_.dt_s;
        }
      }
    }
    if (!fault_) {
      const auto sample = runner_.sample(now, wall);
      if (!sample) {
        stop(EndpointFault::SimulationDeadline);
      } else {
        EndpointPacket p;
        p.receipt = ++sample_sequence_;
        p.received_wall_s = wall;
        p.data = {double(session_),
                  double(sample_sequence_),
                  new_arm ? 1.0 : 0.0,
                  now,
                  now + .025,
                  sample->steering_angles[0],
                  sample->steering_angles[1],
                  sample->steering_angles[2],
                  sample->steering_angles[3],
                  sample->wheel_angular_speeds[0],
                  sample->wheel_angular_speeds[1],
                  sample->wheel_angular_speeds[2],
                  sample->wheel_angular_speeds[3],
                  1};
        targets = guard_.update(now, wall, joints, p);
        if (guard_.fault())
          stop(guard_.fault_reason());
      }
    }
    apply_feedback(now);
    if (fault_)
      targets = guard_.update(now, wall, joints);
    return targets;
  }

private:
  static EndpointLimits limits(const core::Config &c) {
    EndpointLimits l;
    l.max_wheel_speed_radps = c.max_wheel_speed_mps / c.wheel_radius_m;
    l.steering_limit_rad = c.steering_limit_rad;
    l.stopped_wheel_radps = c.stopped_wheel_speed_mps / c.wheel_radius_m;
    return l;
  }
  void apply_feedback(double now) {
    state_.actual_mode = feedback_.actual_mode;
    state_.mode_confirmed = feedback_.confirmed && !fault_;
    state_.mode_fault = feedback_.fault || fault_;
    state_.mode_request_id = feedback_.request_id;
    state_.accepted_mode_request = feedback_.accepted_mode_request;
    state_.time_in_mode_s =
        feedback_.time_in_mode_s + std::max(0.0, now - feedback_stamp_);
  }
  bool arm(const ChassisPacket &p, double now, double wall,
           const ChassisContext &context) {
    const auto &e = p.envelope;
    const auto &c = *e.command.command;
    if (e.session_id <= session_ || e.session_id >= (std::uint64_t{1} << 53) ||
        e.sequence == 0 || c.mode_request || c.target_velocity.vx != 0 ||
        c.target_velocity.vy != 0 || c.target_velocity.wz != 0 ||
        !context.valid || !e.source_task ||
        !e.source_task->matches(context.input) ||
        !core::is_stopped(state_, config_))
      return false;
    if (c.mode != core::DriveMode::DualAckermann &&
        c.mode != core::DriveMode::Crab && c.mode != core::DriveMode::Spin)
      return false;
    // Recovery declares a stationary canonical geometry, never infers mode from
    // a velocity. Noncanonical steering must first be recovered deliberately.
    const auto canonical = core::DriveModel(config_).steering_for_mode(
        c.mode, state_.steering_angles);
    if (session_ != 0 && c.mode != state_.actual_mode)
      return false;
    if (session_ == 0) {
      for (std::size_t i = 0; i < 4; ++i)
        if (std::abs(canonical[i] - state_.steering_angles[i]) >
            config_.steering_tolerance_rad)
          return false;
    }
    auto recovered = state_;
    recovered.actual_mode = c.mode;
    recovered.mode_fault = false;
    recovered.mode_confirmed = true;
    recovered.accepted_mode_request.reset();
    recovered.time_in_mode_s = 0;
    if (executor_)
      executor_->reset(recovered, e.session_id);
    else
      executor_ =
          std::make_unique<core::TimedExecutor>(config_, e.session_id, c.mode);
    runner_.reset(recovered);
    state_ = recovered;
    feedback_ = {c.mode, true,        false, recovered.mode_request_id,
                 0,      std::nullopt};
    feedback_stamp_ = now;
    session_ = e.session_id;
    sequence_ = 0;
    next_tick_s_ = now;
    fault_ = false;
    (void)wall;
    return true;
  }
  core::Config config_;
  core::Kinematics kinematics_;
  core::ProfileRunner runner_;
  EndpointGuard guard_;
  std::unique_ptr<core::TimedExecutor> executor_;
  core::VehicleState state_;
  core::ModeFeedback feedback_;
  core::TransitionPhase phase_ = core::TransitionPhase::Fault;
  core::TimingError timing_error_ = core::TimingError::None;
  core::ExecutionSafetyError safety_error_ = core::ExecutionSafetyError::None;
  std::optional<ChassisPacket> pending_;
  bool fault_ = true;
  std::uint64_t session_ = 0, sequence_ = 0, sample_sequence_ = 0, receipt_ = 0;
  double last_now_ = -1, last_wall_ = -1, next_tick_s_ = -1,
         feedback_stamp_ = 0;
};
} // namespace swerve_gazebo_sim
