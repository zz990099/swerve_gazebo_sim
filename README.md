# Swerve chassis simulation

Independent four-module chassis for ROS 2 Humble / Gazebo Fortress and ROS 2
Jazzy / Gazebo Harmonic. The simulator accepts planar body velocity and explicit
mode requests. It reports measured chassis state and encoder odometry.

There is no planner, path message, obstacle model, trajectory critic or external
algorithm dependency. The C++ plugin implements chassis kinematics, actuator
limits, mode transitions and command expiry. Python only observes encoder
odometry and assembles launch configuration.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select swerve_gazebo_sim
source install/setup.bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py
```

For Humble/Fortress, source Humble and use `demo_ign.launch.py`.
`spawn_gz.launch.py` and `spawn_ign.launch.py` attach to an existing matching world.
Use `headless:=true` for server-only demo operation. Namespaces and automatic
joint/frame prefixes support multiple robots; each robot needs one command owner.

Version 0.7 intentionally replaces the previous interfaces. Old messages,
automatic mode selection, direct joint command topics and algorithm launch
options are removed. Configuration accepts only the current schema.

Read [the command contract](docs/CHASSIS_INTERFACE.md) before sending commands.
Startup is disarmed: a fresh stopped reset transaction is required.
See [architecture](docs/ARCHITECTURE.md) and [validation](docs/VALIDATION.md).
