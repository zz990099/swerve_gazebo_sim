# Chassis interface

Topics are relative to the robot namespace. Joint order is FL, FR, RL, RR.
Coordinates are body x-forward, y-left, positive yaw CCW. All values must be finite.
Commands are planar; unused Twist components must be zero.

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| Input | `chassis_controller/command` | `ChassisCommand` | Body velocity and explicit mode request |
| Output | `chassis_controller/state` | `ChassisState` | Measured joints/twist, modes, transition receipt and fault |
| Output | `joint_states` | `sensor_msgs/JointState` | Steering position and drive velocity |
| Output | `odom` | `nav_msgs/Odometry` | Encoder pose/twist; topic and frames configurable |
| Output | `/tf`, `/tf_static` | TF | Robot transforms; odometry TF independently configurable |
| Input clock | `/clock` | `rosgraph_msgs/Clock` | Generic Gazebo bridge for ROS simulation time |

The joint-controller `steering_controller/commands` and `wheel_controller/commands`
Float64MultiArray topics are internal actuation links. Only the Python chassis
node may publish them. External clients use `ChassisCommand`; ROS does not enforce
exclusive publishing on the internal topics.

Commands use reliable, volatile, depth-one QoS. State uses sensor-data QoS
(best-effort, volatile, depth five). `header.frame_id` must equal the chassis body
frame (default `base_footprint`, with the configured prefix). Command stamps use
simulation time and cannot regress; at most 0.05 s of clock delivery skew is
accepted. State stamps identify the last accepted encoder observation, rather
than publication time or a synchronous physics iteration. Wheel speeds are rad/s.

## Velocity and modes

| Mode | Value | Allowed body velocity |
|---|---|---|
| DualAckermann | 0 | `vy=0`; yaw requires nonzero `vx` |
| Spin | 1 | `vx=vy=0`; command `wz` |
| Crab | 2 | `wz=0`; command `vx,vy` |

Wait for fresh, confirmed state before driving. Startup mode is DualAckermann,
request ID is zero, and entry velocity is zero. There is no separate arming or
session handshake. Publish fresh commands continuously, normally at 50 Hz.

To change mode while Ready, send a `request_id` greater than the accepted ID,
set `mode` to the target and `velocity` to zero. Set `entry_velocity` to a legal
representative velocity in that mode. Entry velocity determines steering geometry
but does not drive the chassis. A zero entry defaults to forward steering for
DualAckermann/Crab and spin geometry for Spin.

The chassis freezes the entry geometry, brakes with steering held, aligns only
after measured wheels stop, and requires measured alignment dwell. Refresh the
same request with zero velocity and unchanged entry throughout. Wait for matching
request ID, actual mode and Ready/confirmed state before commanding motion.
Requests cannot replace an active transition. Subsequent drive/hold commands
retain the accepted request ID and entry velocity. Mode or entry mutation without
a new request is invalid. Large steering changes within a mode also trigger
braking/alignment without changing the selected mode.

Ordinary zero velocity retains mode and steering while decelerating; it does not
request forward alignment. Signed wheel velocity and +/-pi/2 steering implement
reverse directions. Target limits/rates are configured in `config/swerve.yaml`;
actual acceleration also depends on physics and contact.

## Faults and recovery

Defaults expire commands after 0.5 s of simulation age or wall-time silence and
encoder feedback after 0.25 s. Invalid commands, stale feedback and clock rollback
latch a fault. The Python node sends zero wheel targets and holds measured steering;
fault zero bypasses the normal acceleration ramp. While physics is paused, zero
targets can be sent but the physics engine cannot execute motion.

Recover with fresh encoder feedback and a new zero-velocity request with an ID
greater than the last accepted ID, then wait for alignment confirmation. The
request can retain the actual mode. Reusing an old request cannot clear a fault.
Clock rollback clears pose and timestamp history but retains the request high-water
mark. Restarting the Python node resets process-local history; reconnecting clients
must discover the current state.

Phase: Fault=0, Braking=1, Aligning=2, Ready=3. `confirmed` means Ready with no fault.
Fault: None=0, Clock=1, Feedback=2, Command=3, CommandTimeout=4,
TransitionTimeout=5. The accepted entry velocity/steering fields are a frozen
request receipt. Measured velocity and odometry are encoder estimates, not ground
truth or localization; consumers must check freshness themselves.

The watchdog requires a running Python node. See the process-supervision boundary
in [architecture](ARCHITECTURE.md), especially for external Gazebo worlds.
No path, goal, obstacle, clearance or collision decision is made here.
