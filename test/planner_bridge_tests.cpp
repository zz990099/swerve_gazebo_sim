// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/planner_bridge.hpp"
#include <iostream>
#include <stdexcept>
using namespace swerve_gazebo_sim;
void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
void closed_loop(core::Pose2d goal, std::optional<core::DriveMode> expected,
                 std::vector<core::Pose2d> path = {},
                 std::vector<core::CircleObstacle> obstacles = {},
                 core::PathHeadingPolicy heading = core::PathHeadingPolicy::FollowPath) {
  core::Config config;
  config.compute_budget_ratio = 0; // Deterministic offline plant only.
  PlannerBridge planner(config);
  ChassisRuntime runtime(config);
  ChassisContext context;
  context.valid = true;
  context.input.reference_path = path.empty() ? std::vector<core::Pose2d>{{0, 0, 0}, goal} : path;
  context.input.obstacles = obstacles;
  context.input.heading_policy = heading;
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
  require(!expected || runtime.state().actual_mode == *expected,
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
            << " at=" << now << " mode=" << int(runtime.state().actual_mode) << '\n';
}
int main() {
  try {
    ContextHistory history;
    ChassisContext observed;
    observed.valid = true;
    observed.stamp_s = 1;
    observed.input.path_id = 7;
    observed.input.reference_path = {{0, 0, 0}, {1, 0, 0}};
    observed.input.obstacles = {{2, 0, .1}};
    history.push(observed);
    auto newer = observed;
    newer.stamp_s = 1.02;
    newer.input.obstacles[0].radius = .2;
    history.push(newer);
    auto selected = history.at(1.01, newer);
    require(selected && selected->stamp_s == 1 &&
                selected->input.obstacles[0].radius == .1,
            "topic reordering must preserve earlier obstacle observation");
    selected = history.at(1.02, newer);
    require(selected && selected->stamp_s == 1.02 &&
                selected->input.obstacles[0].radius == .2,
            "choose newest compatible original context");
    require(!history.at(1.18, newer), "history cannot renew stale perception");
    auto changed = newer;
    changed.input.reference_path.back().x = 2;
    require(!history.at(1.01, changed), "history cannot revive an older task");
    changed.valid = false;
    history.push(changed);
    require(!history.at(1.02, newer), "invalid context clears retained history");
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
    closed_loop({-.6, 0, 0}, core::DriveMode::DualAckermann);
    std::vector<core::Pose2d> curve;
    std::vector<core::CircleObstacle> obstacles;
    for (int i = 0; i <= 20; ++i) {
      const double a = .5 * i / 20;
      curve.push_back({2 * std::sin(a), 2 * (1 - std::cos(a)), a});
    }
    for (double a : {.18, .42}) {
      for (double offset : {-.9, .9}) {
        obstacles.push_back({(2 + offset) * std::sin(a), 2 - (2 + offset) * std::cos(a), .08});
      }
    }
    closed_loop(curve.back(), std::nullopt, curve, obstacles);
    closed_loop({.5, 0, .4}, core::DriveMode::Spin, {}, {}, core::PathHeadingPolicy::GoalOnly);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
