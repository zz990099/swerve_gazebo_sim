# Architecture

The package has one actuator owner: the `ChassisController` ros2_control plugin.
It converts body velocity to module targets and manages explicit mode transitions
using measured steering and wheel speeds. Its portable runtime has no ROS or
algorithm-library dependency. `command_write.hpp` handles the supported ROS
hardware-interface APIs and detects failed writes.

`StampedGazeboSystem` and `PhysicsClock` bind joint feedback and simulation time
to the same physics iteration. They wrap the Gazebo ros2_control system and do
not run planning, path tracking or collision validation. The wrapper fixes the
position-interface response configuration used by this plant.

The Python odometry observer consumes encoder samples, reconstructs body twist
and integrates pose. It neither subscribes to velocity commands nor writes joints.
No ground truth enters chassis control or odometry. An optional Gazebo truth
publisher exists only for independent physical acceptance.

URDF/Xacro defines mass, inertia, geometry, contact and joint limits. Software
steering travel is +/-pi/2. The physics hard stops are 0.02 rad beyond each
operating endpoint, avoiding the known Fortress/DART hard-stop lock
([upstream issue](https://github.com/ros-controls/gz_ros2_control/issues/165)).
This plant margin does not expand admitted steering commands or relax mode
confirmation. A hardware model with hard stops exactly at +/-pi/2 needs its own
validated actuator model. Launch
builds one strict controller configuration, spawns the robot, activates the joint
state broadcaster and chassis plugin, and starts the observer. The generic bridge
configuration defaults to clock transport and can be replaced for external sensors.

The package exports only ChassisCommand and ChassisState custom messages. Paths,
obstacles, tasks and planner diagnostics belong outside this repository. A planner
adapter translates its body-command output to this contract and consumes measured
state. The simulator does not build, import or check out a planner repository.
