# Swerve chassis simulation

Four-module chassis for ROS 2 Humble / Gazebo Fortress and ROS 2 Jazzy / Gazebo
Harmonic. Its public control interface accepts body velocity and explicit mode
requests, and reports measured chassis state and encoder odometry.

Chassis execution is Python. Gazebo physics and joint actuation use the official
`gz_ros2_control/GazeboSimSystem`, two standard forward command controllers, and
`joint_state_broadcaster`. This repository contains no custom C++ plugins,
physics-time synchronization layer, planner, path messages, obstacle logic or
algorithm dependency.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select swerve_gazebo_sim
source install/setup.bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py
```

For Humble/Fortress, source Humble and use `demo_ign.launch.py`.
Use `headless:=true` for server-only operation. `spawn_gz.launch.py` and
`spawn_ign.launch.py` attach to an existing matching world. Namespaces and automatic
joint/frame prefixes support multiple robots; each robot needs one command owner.
The generic Gazebo bridge defaults to `/clock`; replace its configuration when
additional sensor bridges are needed, or disable it to reuse an existing bridge.

Version 0.8 uses one current command contract. Session IDs, packet sequences,
reset/arming transactions and custom hardware plugins have been removed. Old
message schemas are intentionally unsupported. Configuration is startup-only
and rejects unknown or missing keys.

Read [the command contract](docs/CHASSIS_INTERFACE.md),
[architecture](docs/ARCHITECTURE.md) and [validation](docs/VALIDATION.md).
