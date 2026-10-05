// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/chassis_runtime.hpp"
#include "swerve_mppi/planning/controller.hpp"
#include <iostream>
#include <stdexcept>
using namespace swerve_gazebo_sim;
void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
struct Fixture {
  ChassisRuntime runtime;
  ChassisContext context;
  EndpointTargets joints;
  double now = 1, wall = 10;
  std::uint64_t receipt = 0, sequence = 0, session = 1;
  Fixture() {
    context.valid = true;
    context.input.reference_path = {{0, 0, 0}, {3, 0, 0}};
  }
  core::Output command(core::DriveMode mode, core::Twist2d v = {}) {
    core::Output o;
    o.command = core::ChassisCommand{mode, v, std::nullopt};
    return o;
  }
  ChassisPacket packet(core::Output o, bool arm = false) {
    context.stamp_s = now;
    ChassisPacket p;
    p.receipt = ++receipt;
    p.wall_s = wall;
    p.arm = arm;
    p.envelope = {session,    ++sequence,
                  now,        o,
                  now,        now,
                  now + .025, core::CommandTask::capture(context.input)};
    return p;
  }
  void tick(core::Output o, bool arm = false) {
    const auto p = packet(o, arm);
    joints = runtime.update(now, wall, joints, context, p);
    require(!runtime.fault(), "unexpected command rejection");
    for (int i = 1; i <= 9; ++i) {
      context.stamp_s = now + i * .01;
      joints = runtime.update(now + i * .01, wall + i * .01, joints, context);
      require(!runtime.fault(), "high rate profile failed");
    }
    now += .1;
    wall += .1;
  }
};
void test_jittered_snapshot_admission() {
  Fixture f;
  auto arm = f.packet(f.command(core::DriveMode::DualAckermann), true);
  f.runtime.update_snapshot(1, 10, {}, f.context, &arm);
  auto command = f.packet(f.command(core::DriveMode::DualAckermann));
  command.envelope.source_stamp_s = 1.02;
  command.envelope.issued_at_s = 1.02;
  command.envelope.execute_at_s = 1.1;
  command.envelope.valid_until_s = 1.125;
  command.wall_s = 10.02;
  for (int step = 1; step <= 10; ++step) {
    const double now = 1 + step * .01;
    f.context.stamp_s = now - .02; // Genuine bounded observation delay.
    f.runtime.update_snapshot(now, 10 + step * .01, {}, f.context,
                              step >= 4 ? &command : nullptr);
    require(!f.runtime.fault(), "bounded queue jitter must preserve admission");
  }
  require(std::abs(f.runtime.context_age() - .02) < 1e-9 &&
              std::abs(f.runtime.command_source_age() - .08) < 1e-9 &&
              std::abs(f.runtime.command_receipt_age() - .08) < 1e-9 &&
              std::abs(f.runtime.command_schedule_slack() - .06) < 1e-9,
          "diagnostics must retain source, receipt and application clocks");
  auto late = command;
  late.receipt += 1;
  late.envelope.sequence += 1;
  late.envelope.source_stamp_s = 1.11;
  late.envelope.issued_at_s = 1.11;
  late.envelope.execute_at_s = 1.2;
  late.envelope.valid_until_s = 1.225;
  late.wall_s = 10.11;
  f.context.stamp_s = .9;
  f.runtime.update_snapshot(1.11, 10.11, {}, f.context, &late);
  require(f.runtime.fault() && f.runtime.rejection_detail() == 3,
          "fresh packets must not hide stale independent context");
}
int main() {
  try {
    test_jittered_snapshot_admission();
    Fixture epsilon;
    epsilon.joints.wheels.fill(1e-6);
    epsilon.tick(epsilon.command(core::DriveMode::DualAckermann), true);
    require(!epsilon.runtime.fault(),
            "stationary encoder roundoff must not reject zero-wheel arm");
    Fixture f;
    f.tick(f.command(core::DriveMode::Crab), true);
    require(f.runtime.prediction().has_value() &&
                std::abs(f.runtime.prediction()->stamp_s -
                         f.runtime.next_tick()) < 1e-9,
            "installed profile must expose its original prediction interval");
    for (int i = 0; i < 5; ++i)
      f.tick(f.command(core::DriveMode::Crab, {.2, 0, 0}));
    require(f.runtime.state().actual_mode == core::DriveMode::Crab &&
                f.joints.wheels[0] > 0,
            "forward Crab must not infer Ackermann");
    for (int i = 0; i < 6; ++i)
      f.tick(f.command(core::DriveMode::Crab));
    require(f.runtime.state().actual_mode == core::DriveMode::Crab,
            "zero must retain mode");
    auto switch_command = f.command(core::DriveMode::Spin);
    switch_command.command->mode_request =
        core::ModeRequest{1, core::DriveMode::Spin, {0, 0, .2}};
    bool confirmed = false;
    for (int i = 0; i < 18; ++i) {
      f.tick(switch_command);
      for (double speed : f.joints.wheels)
        require(speed == 0, "request cannot authorize Drive");
      if (f.runtime.state().mode_confirmed) {
        confirmed = true;
        break;
      }
    }
    require(confirmed && f.runtime.state().actual_mode == core::DriveMode::Spin,
            "explicit mode handshake failed");
    require(f.runtime.state().accepted_mode_request &&
                f.runtime.state().mode_request_id == 1,
            "accepted request receipt missing");
    const auto angles = f.joints.steering;
    f.tick(f.command(core::DriveMode::Spin));
    require(f.joints.steering == angles,
            "zero must not return steering to zero");
    f.tick(f.command(core::DriveMode::Spin, {0, 0, .2}));
    // Same-ID mutation must fault, never silently retarget.
    switch_command.command->mode_request->entry_velocity.wz = .3;
    auto changed = f.packet(switch_command);
    f.runtime.update(f.now, f.wall, f.joints, f.context, changed);
    require(f.runtime.fault(), "mutated request must latch");
    require(!f.runtime.prediction(),
            "fault cannot retain an authorized prediction");

    // A large physical body must use the same bound in planning and execution.
    core::Config large;
    large.robot_radius_m = .72;
    large.compute_budget_ratio = 0;
    ChassisContext occupied;
    occupied.valid = true;
    occupied.stamp_s = 1;
    occupied.input.reference_path = {{0, 0, 0}, {1, 0, 0}};
    occupied.input.obstacles = {{.65, 0, .03}};
    occupied.input.vehicle.stamp_s = 1;
    require(
        !core::Controller(large).compute(occupied.input).command,
        "large body cannot authorize motion through its occupied footprint");
    ChassisRuntime large_runtime(large);
    auto large_arm = f.packet(f.command(core::DriveMode::DualAckermann), true);
    large_arm.envelope = {
        9, 1, 1,     f.command(core::DriveMode::DualAckermann),
        1, 1, 1.025, core::CommandTask::capture(occupied.input)};
    large_arm.wall_s = 10;
    large_runtime.update(1, 10, {}, occupied, large_arm);
    require(large_runtime.fault() && !large_runtime.prediction(),
            "execution cannot arm inside the large physical footprint");

    Fixture lost;
    lost.tick(lost.command(core::DriveMode::DualAckermann), true);
    lost.context.stamp_s = lost.now;
    auto stopped =
        lost.runtime.update(lost.now, lost.wall, lost.joints, lost.context);
    require(lost.runtime.fault(), "missing next command must stop");
    for (double v : stopped.wheels)
      require(v == 0, "fault must zero wheels");
    auto stale =
        lost.packet(lost.command(core::DriveMode::DualAckermann), true);
    lost.runtime.update(lost.now, lost.wall, lost.joints, lost.context, stale);
    require(lost.runtime.fault(), "same session cannot recover");
    ++lost.session;
    lost.tick(lost.command(core::DriveMode::DualAckermann), true);
    require(!lost.runtime.fault(), "new stopped session recovery failed");
    // A fresh wall watchdog must reject pause/resume before accepting a
    // command.
    lost.wall += .2;
    auto late =
        lost.packet(lost.command(core::DriveMode::DualAckermann, {.2, 0, 0}));
    lost.runtime.update(lost.now, lost.wall, lost.joints, lost.context, late);
    require(lost.runtime.fault(),
            "wall silence must not be hidden by late command");

    Fixture mismatch;
    mismatch.tick(mismatch.command(core::DriveMode::DualAckermann), true);
    auto old = mismatch.packet(
        mismatch.command(core::DriveMode::DualAckermann, {.2, 0, 0}));
    mismatch.context.input.path_id = 99;
    mismatch.runtime.update(mismatch.now, mismatch.wall, mismatch.joints,
                            mismatch.context, old);
    require(mismatch.runtime.safety_error() ==
                core::ExecutionSafetyError::TaskMismatch,
            "task identity must be checked independently");
    Fixture cancel;
    cancel.tick(cancel.command(core::DriveMode::DualAckermann), true);
    auto p = cancel.packet({});
    cancel.runtime.update(cancel.now, cancel.wall, cancel.joints,
                          cancel.context, p);
    require(cancel.runtime.fault(), "absent authorization must latch");
    Fixture context;
    context.tick(context.command(core::DriveMode::DualAckermann), true);
    p = context.packet(context.command(core::DriveMode::DualAckermann));
    context.context.stamp_s = context.now - .2;
    context.runtime.update(context.now, context.wall, context.joints,
                           context.context, p);
    require(context.runtime.fault(), "stale context must fail closed");
    std::cout << "Explicit velocity/mode, receipts, zero, timeout, task, "
                 "cancellation and recovery passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
