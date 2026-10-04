// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/planner_bridge.hpp"
#include <iostream>
#include <stdexcept>
using namespace swerve_gazebo_sim;
void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
void closed_loop(core::Pose2d goal, core::DriveMode expected) {
  core::Config config;
  config.compute_budget_ratio = 0; // Deterministic offline plant only.
  PlannerBridge planner(config);
  ChassisRuntime runtime(config);
  ChassisContext context;
  context.valid = true;
  context.input.reference_path = {{0, 0, 0}, goal};
  context.input.path_id = 41;
  EndpointTargets joints;
  double now = 1, wall = 10;
  context.stamp_s = now;
  runtime.update(now, wall, joints, context);
  auto feedback = [&] {
    return PlannerFeedback{runtime.state(), runtime.fault(), runtime.session(),
                           runtime.next_tick()};
  };
  auto arm = planner.arm(feedback(), context, 1);
  require(arm && arm->arm, "stopped explicit arm required");
  arm->receipt = 1;
  arm->wall_s = wall;
  joints = runtime.update(now, wall, joints, context, arm);
  require(!runtime.fault(), "arm rejected");
  std::uint64_t receipt = 1;
  bool complete = false;
  std::optional<ChassisPacket> queued;
  for (int step = 0; step < 4000; ++step) {
    auto packet = planner.plan(feedback(), context);
    require(planner.phase() == PlannerPhase::Running,
            "planner fault in closed loop");
    if (packet) {
      require(packet->envelope.source_stamp_s == runtime.state().stamp_s,
              "must retain original observation time");
      require(packet->envelope.source_task->path_id == 41,
              "must retain originating task");
      packet->receipt = ++receipt;
      packet->wall_s = wall;
      queued = packet;
      require(!planner.plan(feedback(), context),
              "one plan per execution boundary");
      if (planner.output().goal_reached) {
        complete = true;
        break;
      }
    }
    now = 1 + (step + 1) * .01;
    wall = 10 + (step + 1) * .01;
    context.stamp_s = now;
    joints = runtime.update(now, wall, joints, context, queued);
    queued.reset();
    if (runtime.fault()) {
      std::cerr << "step=" << step << " t=" << now
                << " endpoint=" << int(runtime.reason())
                << " timing=" << int(runtime.timing_error())
                << " safety=" << int(runtime.safety_error()) << '\n';
      throw std::runtime_error("execution rejected delayed MPPI plan");
    }
  }
  require(complete, "MPPI did not complete within 40 simulation seconds");
  require(runtime.state().actual_mode == expected,
          "expected explicit terminal mode");
  require(core::is_stopped(runtime.state(), config),
          "completion requires stopped feedback");
  require(std::hypot(runtime.state().pose.x - goal.x,
                     runtime.state().pose.y - goal.y) <= .06,
          "measured goal tolerance");
  // A stale independent context must fault without another authorized plan.
  auto f = feedback();
  f.vehicle.stamp_s += .2;
  f.next_execute_at += .2;
  require(!planner.plan(f, context) && planner.phase() == PlannerPhase::Fault,
          "stale context must latch");
  require(!planner.arm(f, context, 1), "old session cannot recover");
  context.stamp_s = f.vehicle.stamp_s;
  require(planner.arm(f, context, 2).has_value(),
          "explicit stopped new-session recovery");
  std::cout << "PASS goal=" << goal.x << ',' << goal.y << ',' << goal.yaw
            << " at=" << now << " mode=" << int(expected) << '\n';
}
int main() {
  try {
    // ROS issuance can be newer than transported physics feedback. Wait for a
    // source strictly newer than the arm issuance, avoiding command replay.
    PlannerBridge protocol;
    PlannerFeedback f;
    f.vehicle.stamp_s = 1;
    ChassisContext c;
    c.valid = true;
    c.stamp_s = 1;
    c.input.reference_path = {{0, 0, 0}, {.6, 0, 0}};
    auto arm = protocol.arm(f, c, 5, 1.01);
    require(arm && arm->envelope.source_stamp_s == 1 &&
                arm->envelope.issued_at_s == 1.01 &&
                arm->envelope.execute_at_s == 1.01,
            "arm preserves source independently from issuance");
    f.session = 5;
    f.fault = false;
    f.vehicle.stamp_s = 1.01;
    f.next_execute_at = 1.11;
    c.stamp_s = 1.01;
    require(!protocol.plan(f, c), "arm acknowledgement cannot replay issuance");
    protocol.stop();
    require(!protocol.arm(f, c, 5), "recovery must increase session");
    require(!protocol.arm(f, c, 6, 1.2), "excessively old source cannot arm");
    closed_loop({.6, 0, 0}, core::DriveMode::DualAckermann);
    closed_loop({0, .6, 0}, core::DriveMode::Crab);
    closed_loop({0, 0, .7}, core::DriveMode::Spin);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
