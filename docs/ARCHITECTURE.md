# Architecture

`controller.py` is the single Python command owner. It validates velocity/mode
commands, consumes measured `joint_states`, publishes joint targets and chassis
state, and integrates encoder odometry. `chassis.py` holds ROS-independent actuator
limits and the small braking/alignment state machine. `kinematics.py` provides
inverse/forward kinematics and pose integration without NumPy.

Gazebo loads the official `gz_ros2_control/GazeboSimSystem` hardware. Standard
forward command controllers expose steering position and wheel velocity, while
`joint_state_broadcaster` reports encoders. Steering servo gain is configured
through the official plugin's `position_proportional_gain` parameter. No custom
hardware wrapper, controller plugin, physics-clock plugin or lifecycle protocol
is built here. ROS interface generation still uses the usual rosidl toolchain.

The Python timer uses steady time at 100 Hz; target ramps and mode deadlines use
ROS simulation time. Source-stamp and wall-time watchdogs reject stale commands
and encoders, including when `/clock` pauses. State stamps refer to the last
accepted encoder sample. They are not an atomic physics-step snapshot. Steering
and wheel targets travel through separate DDS topics; Python timing is best effort.
This is a simple chassis plant for algorithm validation, not a deterministic
hardware execution loop.

While Python is running, invalid/expired input commands zero wheel targets and
holds measured steering. On ordinary shutdown it sends a final zero target.
A killed or stalled Python process cannot execute its watchdog, and standard
forward controllers retain their last target. Launch shuts down when the chassis
process exits: the independent demo consequently closes its Gazebo world. When
attaching to an externally managed world, the world owner must provide process
supervision and stop/teardown behavior. Launch does not stop an external world.

URDF/Xacro defines mass, inertia, geometry, contact and joint limits. Commanded
steering travel is +/-pi/2. Physical hard stops include a 0.02 rad margin to avoid
Fortress/DART endpoint locking
([upstream issue](https://github.com/ros-controls/gz_ros2_control/issues/165)).
This margin does not expand allowed steering commands. Optional Gazebo ground
truth is used only by acceptance tests; it never enters execution or odometry.

Launch generates strict configuration, spawns the chassis, activates three
standard ros2_control controllers, then starts Python execution. A single generic
bridge defaults to clock transport and can also bridge external sensors. The
package exports only `ChassisCommand` and `ChassisState` custom messages. A planner
adapter owns paths, obstacles, goals and prediction and translates its output to
the chassis command contract. Nothing in this package imports or builds a planner.
