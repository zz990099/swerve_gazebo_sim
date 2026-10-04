// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/chassis_runtime.hpp"
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
int main() {
  try {
    Fixture epsilon;
    epsilon.joints.wheels.fill(1e-6);
    epsilon.tick(epsilon.command(core::DriveMode::DualAckermann), true);
    require(!epsilon.runtime.fault(),
            "stationary encoder roundoff must not reject zero-wheel arm");
    Fixture f;
    f.tick(f.command(core::DriveMode::Crab), true);
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
