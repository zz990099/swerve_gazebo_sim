# Protected external joint endpoint (0.2)

Launch either Gazebo family with `external_joint_control:=true`. Bringup starts
one guarded ros2_control plugin instead of steering_controller/wheel_controller;
the Python node publishes only encoder odometry and health. The plugin owns all
eight joint command interfaces, preventing concurrent default and external owners.
It is independent of the MPPI/publisher process. Configuration is fixed after first
configure; restart the controller process to change guard limits.

## Wire contract

Publish `std_msgs/msg/Float64MultiArray` to
`<namespace>/guarded_joint_controller/commands` with empty layout and exactly 14
finite values. Use volatile QoS, depth 1 (the subscriber is best effort). No retained
or replayed command can arm the endpoint. Units and order are fixed:

| Index | Meaning |
| --- | --- |
| 0 | Positive integer session, strictly newer for arm/recovery |
| 1 | Positive strictly increasing integer sequence within a session |
| 2 | arm: exactly 1 for explicit startup/recovery, otherwise 0 |
| 3 | Original sample time in simulation seconds; cannot be in the future |
| 4 | Simulation valid-until time, greater than sample time |
| 5–8 | Steering positions in rad, FL FR RL RR |
| 9–12 | Wheel angular velocities in rad/s, FL FR RL RR |
| 13 | Protocol version: exactly 1 |

Integers must be exactly representable and less than 2^53. Session/sequence must
never be inferred from arrival time. A higher sequence does not authorize a new
session or revive an expired stream. Maximum validity is 0.03 simulation seconds;
wall timeout is 0.10 steady-clock seconds. Do not lengthen validity to hide planner
latency. These defaults require an independently scheduled high-rate sampler.
Normal commands may contain ONLY actual ProfileRunner samples from admitted MPPI
profiles. The endpoint bounds joint targets but does not revalidate obstacles,
body acceleration, mode semantics, or the profile's source task.

`guarded_joint_controller/status` publishes UInt64MultiArray
`[accepted_session, accepted_sequence, fault(0/1), reason]` outside the real-time update.
Reason codes are 0 none, 1 disarmed, 2 clock, 3 feedback, 4 simulation deadline,
5 wall watchdog, 6 rejected command and 7 interface write failure. Fault transitions
also log sample/deadline/current simulation times and wall receipt age outside the
real-time update. It is acknowledgement/diagnostic data, not vehicle-mode confirmation. Check its
freshness and match the session; stopped encoders alone do not prove an arm was
accepted. Invalid commands can fault the endpoint even if they do not advance the
last accepted session/sequence. This protocol is not an authentication boundary.

## Fault and recovery

Simulation expiry, wall expiry, backward/invalid clocks, invalid feedback,
malformed packets, replay, wrong session, inadmissible targets or reported
command-interface write failure latch a stop. On a write failure the controller
immediately retries stopped targets and continues retrying on subsequent ticks.
Expiry is checked before accepting a newer target, so a late packet cannot conceal
a missed deadline. Every update while faulted commands zero wheels and holds
bounded measured steering; corrupt angles never propagate NaN to command interfaces.
The guard remains in control while faulted, rather than releasing the interfaces.
Deactivation also writes zero wheels; session high-water marks survive lifecycle
reconfiguration/reactivation of that plugin instance.
Configure, activation and deactivation refresh the status snapshot without needing
a control update. An inactive controller therefore reports Disarmed while its wall
timer keeps publishing, instead of repeating an old healthy acknowledgement.
Reported lifecycle command write failures publish WriteFailure. Reactivation alone
never arms the endpoint; a new stopped-session handshake remains required.

Arm requires a new session, finite measured wheels <= stopped_wheel_speed (default
0.05 rad/s), exactly zero requested wheel speeds, and steering within 0.05 rad of
measured positions. Repeat the identical frozen zero-wheel arm packet
while awaiting acknowledgement; unchanged retries before any later accepted
sequence are no-ops and do not extend either deadline. After acknowledgement,
stream fresh samples. If delivery is lost or the validity deadline is
missed, stop, verify the plant, and arm with another newer session. Do not refresh
an arm timestamp, revive an expired same-session arm, or stream Drive while awaiting
acknowledgement. An arm in the
same update as a clock rollback is rejected; recover on a later healthy tick.
The MPPI owner must also verify stopped body motion/confirmed mode and reset its
Controller, TimedExecutor and ProfileRunner; this endpoint cannot infer that state.

A paused world does not suspend the wall clock. The guard checks elapsed wall time
on the next ros2_control update and stops before accepting an old-session target.
While physics is paused there is no physical movement; status may reflect the last
update until resume. The response includes controller/physics scheduling delay and
is not a hard wall-time stop guarantee. The control manager must keep updating;
the plugin cannot protect hardware outside its owned interfaces or a failed Gazebo
process. A zero wheel command is an emergency response, not a certified nominal
braking curve or a guarantee of zero physical stopping distance.

## Acceptance tests

The standalone C++ guard test covers publisher silence, paused clock, deadline gaps,
old sessions/replay, explicit stopped recovery, invalid data and clock rollback.
The ROS plugin lifecycle test loads the actual plugin, assigns loaned interfaces,
uses ROS commands/status transport and checks inactive status with no update calls,
reactivation, cleanup/reconfigure and session history on both ROS distributions.
Both ROS/Gazebo CI variants additionally run:

```bash
python3 test/run_simulation.py --gazebo-version gz --external --logs artifacts/external
# Use ign on Humble/Fortress.
```

The probe starts a separate publisher, waits for low-speed physical motion, sends
SIGKILL (no shutdown stop), checks guard fault plus stopped encoders and independent
Gazebo truth, rejects an old-session restart, and verifies a new-session recovery.
The ordinary Twist physical smoke remains enabled. This does not execute MPPI in
Gazebo or calibrate steering, slip, braking and transport uncertainty; those are
next-stage acceptance work.
