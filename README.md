# Continuous four-wheel swerve chassis

ROS 2 / Gazebo chassis simulation with independent steering and reversible drive.
The body command is `(vx, vy, wz)` in `base_footprint`, including simultaneous
translation and rotation. There is no Ackermann / crab / spin mode protocol.

| Package | Responsibility |
| --- | --- |
| `swerve_chassis_core` | ROS-free C++17 kinematics, coordinated setpoints, response, SE(2) integration |
| `swerve_chassis_msgs` | Measured state and complete generator snapshot |
| `swerve_chassis_controller` | C++ ROS velocity execution, watchdogs and encoder odometry |
| `swerve_gazebo_sim` | Python launch/configuration, eight-joint URDF, worlds, physical tests |

## Build and run

Clone this repository beneath a ROS workspace's `src/`, then:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to swerve_gazebo_sim --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py
```

Use `demo_ign.launch.py` for Fortress. The `spawn_gz.launch.py` and
`spawn_ign.launch.py` entry points attach to an existing world, with namespace,
robot name, frame/joint prefix and independent odometry publication options.
The existing eight physical joints and standard Gazebo ros2_control hardware
remain the motion source; the controller never sets the body's pose directly.

Configuration: `swerve_gazebo_sim/config/swerve.yaml`. Geometry and limits are
startup-only and shared by URDF, controller and the published prediction state.
The default command is `geometry_msgs/msg/TwistStamped` on `cmd_vel`, with current
ROS time and `header.frame_id = base_footprint` (including configured prefix).
Send a fresh zero command after startup/fault recovery, then continuously stream
commands at e.g. 50 Hz. A publisher must advance stamps, and use simulation time
when the simulation does. See `swerve_gazebo_sim/test/smoke_chassis.py` for a stamped publisher. For simple
CLI experiments or Humble Nav2, set `control.command_type: unstamped` in a copy
of the YAML, pass `config:=/absolute/path/to/copy.yaml`, then:

```bash
# First recover/start with zero, then replace with a streaming motion command.
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{}'
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3, y: 0.15}, angular: {z: 0.25}}'
```

Select exactly one input type; the node never subscribes to both.

**Breaking version 1.0:** old `swerve_gazebo_sim/ChassisCommand` and mode state
messages are removed. The existing MPPI main is not compatible yet; its next
migration must depend on the core and new messages. Pin the old sim commit
`9358c96884d7fa4866cea7c053daa7a7cec82d8f` to reproduce the old pair.

[Architecture](docs/ARCHITECTURE.md) · [Interface](docs/CHASSIS_INTERFACE.md) ·
[Validation](docs/VALIDATION.md)
