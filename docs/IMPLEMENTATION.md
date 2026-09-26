# Initial implementation

Package and directory name: `swerve_gazebo_sim`.

1. Establish a single ROS package with public dependencies, Apache-2.0 licensing,
   a standalone build and no workspace-specific imports or paths.
2. Create a parameterized primitive chassis with four steering modules, physical
   inertia and wheel contact, plus a bundled offline empty world.
3. Implement planar inverse/forward kinematics, bounded steering and wheel commands,
   command/feedback watchdogs and encoder-derived odometry.
4. Add sequenced launch files with per-robot names, controller parameters and an
   optional Gazebo ground-truth bridge used only for validation.
5. Build in a workspace containing only this package, run analytic and model tests,
   then verify physical motions and two-robot isolation in Gazebo.

The first release uses a Python controller above standard ros2_control forward
controllers. Kinematics is a separate ROS-independent module to allow later reuse.
There is no real hardware adapter, navigation stack, application task layer or
sensor payload in the package. Those can consume its standard ROS interfaces.

Acceptance: the package can be moved to a separate source repository and built in
an otherwise empty workspace; its own demo spawns a chassis that drives forward,
backward, sideways, spins, combines translation/rotation and stops on command expiry.
Two instances must activate all controllers, publish fresh odometry with separate
TF frames, and respond only to commands in their own namespaces.
