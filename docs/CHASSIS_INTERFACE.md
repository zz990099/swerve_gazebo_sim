# Chassis interface v1

All relative topics are resolved in the robot namespace. `base_footprint` is
prefixed with `frame_prefix`; input is in the body frame, no TF conversion occurs.

| Topic | Type | Contract |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/TwistStamped` | Default; current ROS stamp, exact body frame, x/y/yaw rates |
| `cmd_vel` | `geometry_msgs/Twist` | Only when startup `command_type=unstamped`; receive time/body frame implied |
| `joint_states` | `sensor_msgs/JointState` | Each of eight named joints exactly once; steering position and drive velocity required |
| `steering_controller/commands` | `std_msgs/Float64MultiArray` | FL, FR, RL, RR angle targets, rad |
| `wheel_controller/commands` | `std_msgs/Float64MultiArray` | FL, FR, RL, RR drive targets, rad/s |
| `chassis_controller/state` | `swerve_chassis_msgs/ChassisState` | Sensor-data QoS; execution and prediction snapshot |
| `odom` | `nav_msgs/Odometry` | Configurable topic/frame, measured encoder estimate |

Only linear.x, linear.y and angular.z may be nonzero. NaN/Inf and other axes
are errors. Stamped commands require a nonnegative, strictly increasing timestamp
no older than `cmd_timeout` and no more than 0.01 s in the future. Commands time
out after `wall_timeout` even if simulation time is paused. Feedback similarly
requires fresh source and receive times within `feedback_timeout`. Fresh zero
commands recover a latched fault only after the measured drive wheels stop.

The state publication header is current ROS time. `feedback_stamp` identifies the
encoder sample, `command_stamp` identifies the last accepted input (receive time
for unstamped input), and `command_sequence` is an ordinary accepted-message
counter, not an acknowledgement protocol.

`steering_angles` / `wheel_speeds` are measured, `velocity` is their fitted body
velocity. `requested_velocity` is the last accepted input, `executable_velocity`
is the coordinated target. Restore `core::State` using `target_steering`,
`target_wheel_speeds`, `generator_valid`. Planning also requires fresh measured
joints and the expected command/control timing; a stale or invalid snapshot
must not be used for moving rollout initialization.

`execution`: 0 tracking, 1 limited, 2 braking, 3 presteering, 4 stopped.
`fault`: 0 none, 1 command/command timeout, 2 feedback, 3 clock/control time gap.
`command_fresh`, `feedback_valid` are independent freshness indicators.
`consistency_residual` is RMS wheel-vector disagreement in m/s.

`geometry` order: wheelbase, track_width, wheel_radius (meters).
`limits` order: steering limit, steering rate, wheel speed, wheel acceleration,
linear speed, angular speed, linear acceleration, angular acceleration,
steering tracking tolerance, stopped wheel speed. These are the exact startup
parameters used by the core. Downstream MPPI must compare them with its model
configuration before accepting a snapshot. No second independent set of limits
should silently override them.

The former mode, requested mode, entry velocity, request ID, confirmed and
accepted steering fields have no compatibility adapter. Update consumers to
standard velocity + measured/prediction state. Select one message type at startup.
