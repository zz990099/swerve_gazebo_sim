# Chassis interface

All topic names below are relative to the robot namespace. Joint order is
FL, FR, RL, RR. Coordinates are body x-forward, y-left, positive yaw CCW.
All values must be finite. Commands are planar; unused Twist components are zero.

| Direction | Topic | Type | Semantics |
|---|---|---|---|
| Input | `chassis_controller/command` | `ChassisCommand` | Atomic body velocity and explicit mode request |
| Output | `chassis_controller/state` | `ChassisState` | Measured joints/body twist, actual/requested mode, phase, receipt and fault |
| Output | `joint_states` | `sensor_msgs/JointState` | Encoder position (rad) and wheel velocity (rad/s) |
| Output | `odom` | `nav_msgs/Odometry` | Encoder-derived pose/twist; configurable topic and frames |
| Output | `/tf`, `/tf_static` | TF | Robot transforms; odometry TF independently configurable |
| Input clock | `/clock` | `rosgraph_msgs/Clock` | Generic Gazebo bridge for ROS observers and clients |

Commands use reliable, volatile, depth-one QoS. Chassis state is best-effort,
volatile, depth one. Subscribe with compatible QoS. There is no second actuator
command entry point. `header.frame_id` must equal the configured body frame
(default `base_footprint`, prefixed when applicable). Header stamps are simulation
time. State stamps use the physics clock paired with the joint read, independently
of delayed ROS `/clock` callbacks. Wheel speed in state is **rad/s**, not m/s.

## Sessions and expiry

1. Wait for fresh measured state and stopped wheels. Startup mode is DualAckermann.
2. Send a reset command with `reset=true`, a session ID greater than the last
   accepted session, nonzero sequence, `mode=actual_mode`, request ID zero, and
   zero velocity and entry velocity. Reset does not recenter steering.
3. Wait for the matching session and Ready/confirmed state. Reset is one-shot;
   retries use a newer session. Subsequent packets use `reset=false` and strictly
   increasing sequence numbers in the accepted session.
4. Publish fresh packets continuously, normally at 50 Hz. Defaults expire after
   0.5 seconds in either simulation or wall time. Source stamps cannot regress
   or be future-dated. Replayed sequences, invalid packets and expired commands
   fault the chassis. Fault handling commands zero wheel speed and holds measured
   steering. Normal commands cannot rearm a fault; repeat the stopped reset with
   a newer session.

A simulation pause is detected on the first resumed control update; a paused
physics engine cannot physically execute a stop. Lifecycle deactivate/reactivate
preserves session history and disarms the controller. Reloading the plugin starts
a new process-local session history; clients must discover and reset explicitly.

## Explicit modes

| Mode | Value | Admitted body velocity |
|---|---|---|
| DualAckermann | 0 | `vy=0`; yaw requires nonzero `vx` |
| Spin | 1 | `vx=vy=0`; command `wz` |
| Crab | 2 | `wz=0`; command `vx,vy` |

To switch, while Ready, increment `request_id`, set `mode` to the target,
set `velocity` to zero, and set `entry_velocity` to a representative legal
velocity in that mode. Entry velocity specifies steering geometry, not motion.
The chassis freezes that geometry, brakes with steering held, aligns only after
measured wheels stop, and requires measured alignment dwell before confirmation.
Continue refreshing the same request and unchanged entry velocity throughout.
Wait for matching request ID, actual mode and Ready/confirmed before sending a
nonzero velocity. A replacement request during a transition is rejected.

Retain the accepted request ID and entry velocity in subsequent drive/hold
packets; the returned accepted geometry is the original receipt. Changing the
mode without a new request is invalid. Large steering changes within a mode also
cause an automatic brake/alignment interval; they do not select a different mode.

A zero velocity retains mode and steering while decelerating. It is not a mode
request and does not imply forward alignment. Signed wheel velocities and
steering commands bounded to +/- pi/2 implement reverse directions. The plant
hard stops include a 0.02 rad margin beyond this operating range; see architecture.

## Feedback and limits

Phase: Fault=0, Braking=1, Aligning=2, Ready=3. `confirmed` requires Ready and no
fault. Fault codes: None=0, Disarmed=1, Clock=2, Feedback=3, Command=4,
Timeout=5, Transition=6, Write=7. `velocity` is reconstructed from measured joints.
Encoder odometry is not ground truth or localization. State and odometry consumers
must enforce their own freshness and synchronization requirements.

`config/swerve.yaml` defines chassis geometry, speed/rate limits, transition dwell
and timeouts. Updates run at 100 Hz. Steering, wheel and body target rates are
bounded during normal operation; emergency fault zero commands bypass normal
acceleration limits. These are simulator command rules, not collision or physical
braking guarantees. No path, footprint clearance, goal or obstacle is interpreted.

Mode entry, limits and tracking must be modeled by a planner that needs accurate
prediction. A planner must not assume its prediction controls the actual wheels.
