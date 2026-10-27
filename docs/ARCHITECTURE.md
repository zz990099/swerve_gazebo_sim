# Continuous chassis architecture

## Boundary

Nav2/MPPI produces body-frame velocities; the execution layer converts them to
coordinated wheel commands. Gazebo's contacts and joint hardware produce motion.
Python owns launch/configuration/testing, C++ owns the shared per-step math and
ROS controller. No planner or path follower is implemented here.

`swerve_chassis_core` is a normal CMake C++17 library, independent of ROS, Gazebo
and Eigen. Consumers use `find_package(swerve_chassis_core REQUIRED)` and link
`swerve_chassis_core::swerve_chassis_core`. Headers separate types, kinematics
and setpoint generation. Wheel order is FL, FR, RL, RR; wheel speeds are rad/s.

## Generator contract

`Generator::step(desired, measured, previous, dt)` is deterministic and has no
hidden history. `State` contains the previous target arrays and valid flag.
The current desired velocity carries presteering intent. A continuing lateral
request advances steering even while the commanded wheel speeds remain zero.

The implementation bounds the body request and wheel speeds by common scaling,
then traverses the connected feasible prefix of the line from previous executable
body velocity to the desired velocity. It splits the path at each wheel's vector
component zero and speed extremum. Within these intervals, bounded subdivision
and local refinement constrain steering rate, drive acceleration, body acceleration
and measured steering lag. It does not jump over an infeasible representation
boundary to a later feasible endpoint. The operating steering range is +/-pi/2;
the equivalent reversed drive representation is selected within that range.

If progress is blocked, all drive targets are reduced by a common scale while
holding steering. Once commanded and measured wheels are stopped, the target
steering advances at a bounded rate; a stalled servo cannot accumulate unbounded
target lead. This is an actuator fallback, not a chassis motion mode. A continuous
feasible gradual change of twist stays in motion. Exact zero holds the last angles.

Every nonzero nominal target satisfies the same rigid-body rolling constraint.
An individually rate-limited `response()` models nominal actuator response; its
result may be inconsistent. `residual()` explicitly reports the RMS discrepancy
in m/s. The least-squares `forward()` result of inconsistent measured wheels is
an encoder estimate, not proof of physically achievable motion. MPPI must reject
or conservatively penalize excessive predicted residual and validate against
independent Gazebo truth. This repository does not claim a calibrated tire model.

## Fault boundaries

The ROS node is single-threaded and parameters are startup-only. Simulation-time
age and a steady-clock watchdog are both required. Replayed encoder samples do
not refresh watchdogs. Duplicate/out-of-order stamped commands are rejected.
Invalid input, stale feedback, clock rollback and excessive control gaps issue an
immediate zero-wheel target; these fault stops intentionally bypass normal slew
limits. The last valid measured steering is held within operating limits.
Faults latch until fresh zero command + fresh stopped encoders; motion commands
alone cannot clear a fault. Graceful process termination publishes a final zero.
Unexpected process death still shuts down the isolated demo via launch handlers.

Encoder odometry uses exact constant-twist SE(2) integration between valid samples,
never extrapolates across a feedback gap, and preserves its pose on clock reset.
Independent Gazebo truth is a test-only source, not fed back to this odometry.
