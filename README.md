# swerve_gazebo_sim

A standalone four-wheel steering / four-wheel drive chassis simulation. A simple
box chassis and four cylindrical wheels provide a generic starting point without
vendor meshes or application-specific messages. Each steering module has a bounded
±90° steering joint and a continuous wheel joint. Reversing wheel speed allows
motion in any planar direction, including simultaneous translation and rotation.

## Supported environment

The initial target is Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress (Gazebo Sim 6),
with `gz_ros2_control`, `ros_gz_sim` and `ros_gz_bridge`. This is Gazebo Sim, not
Gazebo Classic. Other ROS / Gazebo combinations have not been validated.

Use a ROS installation with the matching Gazebo integration packages available.
Copy this directory alone into an empty workspace's `src/`, then:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select swerve_gazebo_sim
source install/setup.bash
ros2 launch swerve_gazebo_sim demo.launch.py
```

Headless simulation: add `headless:=true`. The supplied world requires no downloads.
For an existing Fortress world with the Physics and UserCommands systems and one
`/clock` bridge, use `spawn.launch.py world_name:=YOUR_WORLD`.

## Drive the chassis

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.4, y: 0.2}, angular: {z: 0.3}}}'
```

Commands are expressed in `base_footprint`. Empty `header.frame_id` means that
frame; a zero stamp uses reception time. Nonzero stamps must use simulation time.
Other frame names, nonfinite values, expired commands and future stamps (>0.1 s)
are rejected and stop the wheels. Stopping publication stops the wheels after
`cmd_timeout` (default 0.5 simulation seconds). Explicit zero velocity stops them
on the next control tick and holds steering orientation. A paused world also pauses
the watchdog; a wall-clock emergency stop is outside this simulation package.

| Relative interface | Type | Meaning |
|---|---|---|
| `cmd_vel` | `geometry_msgs/TwistStamped` | Body velocity: x, y, yaw rate |
| `joint_states` | `sensor_msgs/JointState` | Eight joints from ros2_control |
| `odom` | `nav_msgs/Odometry` | Encoder-based planar pose and velocity |
| `wheel_controller/commands` | `std_msgs/Float64MultiArray` | Internal wheel rad/s, FL FR RL RR |
| `steering_controller/commands` | `std_msgs/Float64MultiArray` | Internal steering radians, same order |
| `ground_truth/odom` | `nav_msgs/Odometry` | Optional Gazebo model pose, for validation only |

`swerve_controller` owns `odom → base_footprint`; `robot_state_publisher` owns the
remaining robot TF. All frames are published on the shared `/tf` and `/tf_static`
topics with unique frame names. Set `control.publish_odom_tf: false` when an
external estimator owns the odometry TF. Ground truth never broadcasts TF.

## Configuration and multiple robots

`config/swerve.yaml` is the geometry and motion-limit source used by both model
generation and control. Copy it and pass `config:=/absolute/path/custom.yaml`.
All fields are startup parameters; restart after changing geometry. Launch validates
positive dimensions and wheel clearance. The initial default is a 0.60 m wheelbase,
0.50 m track, 0.10 m wheel radius and 0.80 × 0.28 × 0.16 m body.

The URDF has three focused files:

- `urdf/chassis.urdf.xacro` defines the chassis links, joints, limits and inertias. It contains no Gazebo plugins or contact settings.
- `urdf/plugins.urdf.xacro` defines Gazebo wheel contact, `ros2_control` and optional ground-truth plugins.
- `urdf/swerve_drive.urdf.xacro` is the complete model used by launch and other consumers. It loads the same `swerve.yaml` file, so dimensions and limits do not need separate Xacro arguments.

The complete model accepts `config_file`, `prefix`, `namespace`, `controllers_file`,
`publish_ground_truth` and `robot_name`. The launch file supplies these values.

```bash
ros2 launch swerve_gazebo_sim demo.launch.py namespace:=robot1 robot_name:=robot1
# In a second sourced terminal, reuse the running world and its clock:
ros2 launch swerve_gazebo_sim spawn.launch.py namespace:=robot2 robot_name:=robot2 x:=2
```

Robot 1 uses `/robot1/cmd_vel`, `/robot1/odom`, its own controller manager and
`robot1_base_footprint` etc. `prefix:=auto` derives link, joint and frame names
from the namespace; override it explicitly if needed. Use a unique `robot_name`,
namespace and prefix for each robot. The clock bridge belongs to the world and
is started once by `demo.launch.py`, never by `spawn.launch.py`.

Launch also accepts `x`, `y`, `z`, `yaw`, `world_name` and `publish_ground_truth`.
`demo.launch.py` additionally accepts `headless` and an absolute `world` SDF path.
Spawning and controller activation are sequenced by process completion, with a
60-second controller-manager timeout. The controller configuration generated for
each namespace lives in a temporary file and is removed when launch shuts down.

## Motion model and limits

For a wheel at `(x_i, y_i)`, its velocity is `(vx - wz*y_i, vy + wz*x_i)`.
The controller chooses a legal steering angle and signed wheel speed, scales all
wheels together when saturated, limits steering command rate and wheel acceleration,
and reduces wheel drive while steering is misaligned. Feedback loss stops wheel
commands. These measures reduce transients but do not provide real hardware safety.

Odometry uses measured wheel velocities and steering positions with a least-squares
rigid-body fit, then SE(2) integration. It assumes rolling without slip and starts at
zero regardless of spawn pose. Covariance is a configurable nominal diagonal, not
an estimated uncertainty. Simulation resets clear command/odometry state; feedback
gaps are not extrapolated. Reinitialize external consumers after a world reset.

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
integration, invalid commands, watchdogs, model dimensions and namespacing.
The integration runner starts a world, executes the physical smoke test and stops
its processes. Run it in a sourced workspace:

```bash
python3 src/swerve_gazebo_sim/test/run_simulation.py
python3 src/swerve_gazebo_sim/test/run_simulation.py --multi
```

Use a separate ROS domain / Gazebo transport partition if other simulations are
already running. The multi-robot check drives one robot while observing that a
second robot stays still. To probe an already running demo, enable
`publish_ground_truth:=true` and run `test/smoke_motion.py` directly.
See `docs/VALIDATION.md` for the recorded environment and results.

## Source and license

Apache-2.0. The geometry, kinematics, controller and launch files in this package
are newly authored. No vendor meshes or external workspace files are required.
This directory can be copied directly into an independent repository or workspace.
