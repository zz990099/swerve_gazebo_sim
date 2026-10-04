## MPPI ROS planning and explicit control (0.4)

The optional `mppi_planner` node connects the standalone MPPI core to the explicit
chassis execution interface. It consumes independently refreshed task/obstacle
context and measured chassis state, plans ahead of each execution boundary and
requires explicit stopped arming/recovery. See [MPPI_ROS.md](docs/MPPI_ROS.md).

## Explicit velocity and mode control (0.3)

Optional `chassis_control:=true` receives typed body velocity and explicit mode
requests, with measured confirmation and persistent mode on zero velocity.
Build with `SWERVE_BUILD_CHASSIS_CONTROLLER=ON` and installed MPPI core 0.20.
See [CHASSIS_INTERFACE.md](docs/CHASSIS_INTERFACE.md) for build, topics, timing,
startup and recovery. Existing Twist and external-joint modes remain available
as mutually exclusive alternatives.

# swerve_gazebo_sim

A standalone four-wheel steering / four-wheel drive chassis simulation. A simple
box chassis and four cylindrical wheels provide a generic starting point without
vendor meshes or application-specific messages. Each steering module has a bounded
±90° steering joint and a continuous wheel joint. The command interface supports
double-Ackermann driving, pure spin and crab translation in either direction.
Lateral translation combined with yaw is deliberately unsupported.

Version 0.2.1 refreshes protected endpoint status during lifecycle changes.
Inactive/reactivated controllers report Disarmed without needing a control update;
ROS plugin lifecycle regressions run on both supported distributions.

## Supported environment

The launch interface has two explicit Gazebo families:

- `demo_ign.launch.py` / `spawn_ign.launch.py` selects Gazebo Fortress (Gazebo Sim 6),
  `ignition.msgs.*` bridge types and `ignition-gazebo-*` system plugins. This is
  validated with Ubuntu 22.04 and ROS 2 Humble.
- `demo_gz.launch.py` / `spawn_gz.launch.py` selects Gazebo Harmonic (Gazebo Sim 8), `gz.msgs.*`
  bridge types and `gz-sim-*` system plugins. It is intended for a matching
  ROS 2 Jazzy / Harmonic installation.

Both variants use `gz_ros2_control`, `ros_gz_sim` and `ros_gz_bridge`. Gazebo
Classic is not supported.

Use a ROS installation with the matching Gazebo integration packages available.
Copy this directory alone into an empty workspace's `src/`, then:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select swerve_gazebo_sim
source install/setup.bash
ros2 launch swerve_gazebo_sim demo_ign.launch.py
```

For Harmonic:

```bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py
```

Headless simulation: add `headless:=true`. Use this mode for automated tests and
when the container has no GPU device. A graphical Gazebo client without hardware
OpenGL can fall back to CPU software rendering and consume several CPU cores. Pass
the host's GPU/render device into the container when a GUI is required.

The bundled worlds run physics at 200 Hz with a target real-time factor of 1.0.
This provides two physics steps per 100 Hz controller update and avoids publishing
simulation clock updates at the previous 1000 Hz rate. The supplied world requires
no downloads. When `world` is empty, the demo selects `empty_ign.sdf` or
`empty_gz.sdf` for the requested family. For an existing world with the Physics and
UserCommands systems and one `/clock` bridge, use `spawn_ign.launch.py` or
`spawn_gz.launch.py` with `world_name:=YOUR_WORLD`, matching that world's version.

## Drive the chassis

```bash
# Double-Ackermann driving: forward translation with yaw, no lateral velocity.
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.4}, angular: {z: 0.2}}}'

# Spin: pure yaw, no translation.
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {angular: {z: 0.3}}}'

# Crab: translation, no yaw. Negative x/y and pure lateral motion also work.
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.4, y: 0.2}}}'
```

Run one publisher at a time and stop it before trying another mode. A command
with both nonzero lateral velocity and yaw stops the drive and returns steering
home. Pure forward/reverse translation is classified as differential mode.

Commands are expressed in `base_footprint`. Empty `header.frame_id` means that
frame; a zero stamp uses reception time. Nonzero stamps must use simulation time.
Other frame names, nonfinite values, expired commands and future stamps (>0.1 s)
are rejected and stop the wheels. Stopping publication stops the wheels after
`cmd_timeout` (default 0.5 simulation seconds). Explicit zero velocity stops them
on the next control tick. After either stop condition, all steering joints return
to zero at `max_steering_rate`. A paused world also pauses the watchdog; a
wall-clock emergency stop is outside this simulation package.

| Relative interface | Type | Meaning |
|---|---|---|
| `cmd_vel` | `geometry_msgs/TwistStamped` | Body velocity: x, y, yaw rate |
| `joint_states` | `sensor_msgs/JointState` | Eight joints from ros2_control |
| `odom` | `nav_msgs/Odometry` | Encoder-based planar pose and velocity |
| `wheel_controller/commands` | `std_msgs/Float64MultiArray` | Internal wheel rad/s, FL FR RL RR |
| `steering_controller/commands` | `std_msgs/Float64MultiArray` | Internal steering radians, same order |
| `drive_status` | `diagnostic_msgs/DiagnosticArray` | Requested/active mode, phase and blocking reason |

`swerve_controller` owns `odom → base_footprint`; `robot_state_publisher` owns the
remaining robot TF. All frames are published on the shared `/tf` and `/tf_static`
topics with unique frame names. Set `control.publish_odom_tf: false` when an
external estimator owns the odometry TF. Odometry is computed from measured wheel
velocities and steering angles in `controller.py`, without Gazebo truth input.

## Configuration and multiple robots

### External joint control for MPPI

Start with `external_joint_control:=true` (or set `control.external_joint_control`
in YAML). Both Gazebo families and both demo/spawn entry points support it:

```bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py headless:=true external_joint_control:=true
# Fortress: use demo_ign.launch.py with the same arguments.
```

In this mode `swerve_controller` publishes odometry/TF and feedback health only.
It creates no joint command publishers, cmd_vel subscription or motion supervisor;
its status reports `command_owner=external` and does not invent MPPI mode feedback.
One external publisher must own the guarded command stream. Publish FL/FR/RL/RR wheel
rad/s and steering radians from `swerve_mppi::ProfileRunner`, consuming only checked
`TimedExecutor` results at a high actuator rate. Map mode feedback from that executor,
convert measured joint rad/s to wheel m/s, and align observations to the actual
application instant. Match geometry, steering stops and rate limits to this model.
Do not pass MPPI through cmd_vel: the default internal mode inference and ramps
do not implement the MPPI full-tick joint profile.

External mode now replaces both forward controllers with one protected
`swerve_gazebo_sim/GuardedJointController`. Publish sampled joint packets to
`guarded_joint_controller/commands`; the old forward-controller topics are absent
in this mode. The plugin checks simulation deadlines and a steady-clock watchdog
independently of the publishing process, latches zero wheel commands on faults,
and requires measured stopped wheels plus a newer session for recovery. See
[docs/EXTERNAL_ENDPOINT.md](docs/EXTERNAL_ENDPOINT.md) for the fixed wire format,
acknowledgement, lifecycle and scheduling limits. A complete ROS MPPI planning
node and physical braking/slip/latency calibration remain separate integration work.
Encoder odometry is not independent ground truth.

`config/swerve.yaml` is the geometry and motion-limit source used by both model
generation and control. Copy it and pass `config:=/absolute/path/custom.yaml`.
All fields are startup parameters; restart after changing geometry. Launch validates
positive dimensions and wheel clearance. The initial default is a 0.60 m wheelbase,
0.50 m track, 0.10 m wheel radius and 0.80 × 0.28 × 0.16 m body.

The URDF has three focused files:

- `urdf/chassis.urdf.xacro` defines the chassis links, joints, limits and inertias. It contains no Gazebo plugins or contact settings.
- `urdf/plugins.urdf.xacro` defines Gazebo wheel contact, `ros2_control` and optional ground-truth plugins.
- `urdf/swerve_drive.urdf.xacro` is the complete model used by launch and other consumers. It loads the same `swerve.yaml` file, so dimensions and limits do not need separate Xacro arguments.

The complete model accepts `config_file`, `gazebo_version`, `prefix`, `namespace`,
`controllers_file` and `publish_ground_truth`. The fixed-version launch entry point
supplies these values; there is no runtime Gazebo-version launch argument.
One generic node named `bridge` loads the entire `config/bridge_ign.yaml` or
`config/bridge_gz.yaml` list. The default list contains `/clock`; add sensor or
other topic mappings to the same file or pass `bridge_config:=/absolute/path/bridge.yaml`.
No ground-truth odometry is bridged. Relative bridge topics expand under the robot
namespace; absolute topics such as `/clock` remain global. Both demo and spawn
use the same bridge through spawn. Set `start_bridge:=false` when joining a world
that already has the required bridge.

```bash
ros2 launch swerve_gazebo_sim demo_ign.launch.py namespace:=robot1 robot_name:=robot1
# In a second sourced terminal, reuse the running world and its clock:
ros2 launch swerve_gazebo_sim spawn_ign.launch.py namespace:=robot2 robot_name:=robot2 x:=2 start_bridge:=false
```

Robot 1 uses `/robot1/cmd_vel`, `/robot1/odom`, its own controller manager and
`robot1_base_footprint` etc. `prefix:=auto` derives link, joint and frame names
from the namespace; override it explicitly if needed. Use a unique `robot_name`,
namespace and prefix for each robot. Reuse one generic bridge per world rather
than creating duplicate `/clock` publishers.

Launch also accepts `x`, `y`, `z`, `yaw`, `world_name` and
`publish_ground_truth` (Gazebo Transport only, for test measurements). Both demo entry points additionally accept `headless` and an
absolute `world` SDF path. The generic `spawn.launch.py` and `demo.launch.py`
entry points have been removed; choose the matching suffixed entry point.
Spawning and controller activation are sequenced by process completion, with a
60-second controller-manager timeout. The controller configuration generated for
each namespace lives in a temporary file and is removed when launch shuts down.

Odometry outputs can be overridden in either spawn or demo launch:

| Launch argument | Default from YAML | Purpose |
|---|---|---|
| `publish_odom` | `true` | Publish kinematic `nav_msgs/Odometry` |
| `odom_topic` | `odom` | Relative topic uses the robot namespace; absolute topic stays global |
| `publish_odom_tf` | `true` | Publish TF independently of Odometry messages |
| `odom_frame` | Automatic | Odometry header and TF parent frame |
| `odom_child_frame` | Automatic | Odometry child and TF child frame |

Empty launch values use YAML; empty frame values in YAML select `<prefix>odom`
and `<prefix>base_footprint`. Explicit frame names are used unchanged, without
prefixing or a coordinate conversion. The child frame should match the physical
base frame to connect the robot TF tree. Disabling either output does not stop
pose integration, joint feedback processing or motion control.

```bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py \
  publish_odom:=true odom_topic:=wheel/odom publish_odom_tf:=false
ros2 launch swerve_gazebo_sim spawn_ign.launch.py \
  publish_odom:=false publish_odom_tf:=true \
  odom_frame:=local_odom odom_child_frame:=base_footprint
```

## Motion model and limits

`config/swerve.yaml` documents every parameter, its unit and where it is used.
`max_steering_rate` limits steering-angle command changes in rad/s;
`max_wheel_speed` limits drive-wheel speed in rad/s, and
`max_wheel_acceleration` limits drive-wheel command acceleration/deceleration in
rad/s² during active drive. Stops and interlocks command zero immediately.
For aligned pure translation, wheel acceleration times wheel radius gives the
linear acceleration (4 m/s² with the defaults). This is not a general body-level
acceleration cap when steering changes. Independent maximum body linear/yaw
speeds and accelerations, steering acceleration and jerk are not configured by
the current controller. Joint torque limits and tire friction remain model values.

The controller selects an explicit mode from each body command:

- Differential mode: `vy = 0`, with `vx` and optional `wz`. All steering joints
  return to zero before double-Ackermann steering and drive commands are applied.
- Spin mode: only `wz` is nonzero. The four modules form the spin pattern.
- Crab mode: translation includes `vy` and `wz = 0`. All modules are parallel.

Lateral translation and yaw cannot be combined. Such a command stops the drive
and returns steering to zero because it does not belong to any supported mode.

Mode entry uses three stages: braking, alignment and active drive. Drive commands
remain zero until every measured wheel speed is below `stopped_wheel_speed` and
the requested mode has remained stable for `mode_dwell_time`. Steering is held
while braking, then moves toward the new target. Every measured steering angle
must remain within `steering_alignment_tolerance` for
`steering_alignment_duration` before drive resumes. Differential entry targets
zero steering; once active, its Ackermann steering tracks the motion command
without repeatedly gating drive. Spin and crab retain alignment checks, including
changes of crab direction. Wheel acceleration limiting begins after entry.

Linear and angular enter/exit thresholds filter small command noise and provide
hysteresis. The lower exit thresholds prevent chatter around a single boundary.
All timing uses simulation time. Zero commands and watchdogs stop drive immediately;
they do not wait for the mode dwell. Feedback loss invalidates the active mode.
These measures reduce transients but do not provide real hardware safety.

| Configuration | Default | Purpose |
|---|---|---|
| `mode_linear_enter_threshold` / `mode_linear_exit_threshold` | 0.01 / 0.005 m/s | Translation deadband and hysteresis |
| `mode_angular_enter_threshold` / `mode_angular_exit_threshold` | 0.01 / 0.005 rad/s | Yaw deadband and hysteresis |
| `mode_dwell_time` | 0.10 s | Stable mode request before steering |
| `stopped_wheel_speed` | 0.05 rad/s | Measured stop confirmation for every wheel |
| `steering_alignment_duration` | 0.05 s | Stable steering alignment before drive |
| `mode_switch_timeout` | 5.0 s | Overall transition deadline, including retargets |
| `status_publish_rate` | 10 Hz | Diagnostic publication rate |

The transition settings are optional in older configuration files; omitted fields
receive these defaults. Dwell and alignment duration can be zero; exit thresholds
must be below their corresponding enter thresholds. A transition timeout latches
the fault and keeps drive stopped. Send a zero command before requesting motion
again. Inspect `ros2 topic echo /drive_status` (or the robot's namespaced topic)
for the phase and reason; timeouts also emit one error log on fault entry.

Interrupting a transition invalidates the previous active mode. Returning to that
mode, or selecting a third mode, must pass the new entry interlock before drive
commands resume.

In differential mode, a module at `(x_i, y_i)` targets velocity
`(vx - wz*y_i, wz*x_i)`. This produces opposite front/rear steering angles and
individual wheel speeds for double-Ackermann motion. Spin and crab use the same
four-module kinematics for their fixed spin pattern and parallel steering direction.

Odometry uses measured wheel velocities and steering positions with a least-squares
rigid-body fit, then feeds SE(2) integration. It starts at zero regardless of spawn
pose. Covariance is a configurable nominal diagonal, not an estimated uncertainty.
An observed ROS clock rollback clears command/odometry state in either control
mode. Older or repeated JointState stamps within the same clock epoch are ignored
without resetting pose or publishing odometry/TF. Ordering and integration intervals
use integer nanoseconds, including at large time epochs. Feedback gaps are not extrapolated.
Reinitialize external consumers after a world reset.

The Gazebo position/velocity interfaces are idealized actuators. The model includes
mass, inertia, gravity and wheel contact, but no suspension, motor electrical model
or calibrated tire dynamics. Use it for functional motion/controller experiments;
real chassis accuracy requires identification and additional actuator/contact models.

## Validation

```bash
colcon test --packages-select swerve_gazebo_sim --event-handlers console_direct+
colcon test-result --verbose
```

The tests check analytic wheel velocities, reverse motion, yaw odometry, pose
integration, invalid commands, watchdogs, steering alignment interlocks, steering
return-to-home behavior, model dimensions and namespacing.
The integration runner starts a world, executes the physical smoke test and stops
its processes. Run it in a sourced workspace:

```bash
python3 src/swerve_gazebo_sim/test/run_simulation.py
python3 src/swerve_gazebo_sim/test/run_simulation.py --multi
# In a Jazzy/Harmonic workspace:
python3 src/swerve_gazebo_sim/test/run_simulation.py --gazebo-version gz
```

Use a separate ROS domain / Gazebo transport partition if other simulations are
already running. The multi-robot check drives one robot while observing that a
second robot stays still. The probe reads simulator truth directly with
`ign topic` / `gz topic --json-output`; it never bridges or republishes truth to ROS.
To probe an already running demo, enable `publish_ground_truth:=true` and run
`test/smoke_motion.py --gazebo-version ign` (or `gz`) directly.
See `docs/VALIDATION.md` for the recorded environment and results.

GitHub Actions runs the same checks on the two supported ROS/Gazebo pairs:

| ROS | Gazebo | Docker official base image |
|---|---|---|
| Humble | Fortress | `ros:humble-ros-base-jammy` |
| Jazzy | Harmonic | `ros:jazzy-ros-base-noble` |

Every push to main and pull request runs formatting/lint, an isolated colcon
build, all unit/model/launch tests, single-robot physical motions, six direct
mode transitions and interrupted returns. Build,
JUnit and simulation logs are uploaded even on failure. Simulations run headless
with an isolated ROS domain and Gazebo transport partition. Other ROS releases
are not automatically claimed supported: their default Gazebo versions may
differ from the two families this package targets.

## Source and license

Apache-2.0. The geometry, kinematics, controller and launch files in this package
are newly authored. No vendor meshes or external workspace files are required.
This directory can be copied directly into an independent repository or workspace.
