// SPDX-License-Identifier: Apache-2.0
#include "swerve_gazebo_sim/planner_conversion.hpp"
#include <stdexcept>
using namespace swerve_gazebo_sim;
void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
int main() {
  msg::ChassisState state;
  state.header.stamp = stamp(2.03);
  state.actual_mode = 2;
  state.confirmed = true;
  state.request_id = 17;
  state.has_accepted_request = true;
  state.accepted_mode = 2;
  state.accepted_steering.fill(.4);
  state.accepted_entry_velocity.x = .15;
  state.wheel_speeds.fill(.2);
  state.next_execute_at = stamp(2.1);
  const auto feedback = decode(state);
  require(feedback.vehicle.stamp_s == seconds(state.header.stamp),
          "preserve original source time");
  require(feedback.vehicle.wheel_speeds[0] == .2,
          "do not convert linear wheel speeds again");
  require(feedback.vehicle.accepted_mode_request &&
              feedback.vehicle.accepted_mode_request->id == 17 &&
              feedback.vehicle.accepted_mode_request->steering_targets[0] ==
                  .4 &&
              feedback.vehicle.accepted_mode_request->entry_velocity.vx == .15,
          "preserve immutable complete accepted receipt");
  ChassisPacket p;
  p.envelope.session_id = 23;
  p.envelope.sequence = 4;
  p.envelope.source_stamp_s = 2.03;
  p.envelope.issued_at_s = 2.06;
  p.envelope.execute_at_s = 2.1;
  p.envelope.valid_until_s = 2.125;
  p.envelope.command.command = core::ChassisCommand{
      core::DriveMode::Spin,
      {},
      core::ModeRequest{18, core::DriveMode::Spin, {0, 0, .25}}};
  core::ControllerInput input;
  input.path_id = 9;
  input.heading_policy = core::PathHeadingPolicy::GoalOnly;
  input.reference_path = {{1, 2, 0}, {1, 2, .7}};
  p.envelope.source_task = core::CommandTask::capture(input);
  const auto wire = encode(p, "robot_base");
  const auto roundtrip = decode(wire, "robot_base");
  require(!roundtrip.malformed && wire.authorized && wire.has_mode_request,
          "valid typed protocol roundtrip");
  require(
      roundtrip.envelope.source_task->matches(input) &&
          roundtrip.envelope.command.command->mode_request->id == 18 &&
          roundtrip.envelope.command.command->mode_request->entry_velocity.wz ==
              .25,
      "body request and task identity roundtrip");
  p.envelope.command.command.reset();
  const auto cancel = encode(p, "robot_base");
  require(!cancel.authorized &&
              !decode(cancel, "robot_base").envelope.command.command,
          "absent output must remain cancellation, not zero Drive");
}
