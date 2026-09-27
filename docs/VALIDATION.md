# Validation record

Date: 2026-09-25.

Environment: Ubuntu 22.04, ROS 2 Humble, Python 3.10.12, Gazebo Fortress 6.16.0,
`gz_ros2_control` 0.7.17. Only this package was mounted into the build workspace;
the only sourced underlay was `/opt/ros/humble`. No application workspace was
mounted or sourced. The existing local development image supplied public ROS and
Gazebo dependencies; the package does not require that image.

`rosdep check --from-paths <package> --ignore-src` reported that all system
dependencies were satisfied. `colcon build --packages-select swerve_gazebo_sim`
succeeded. The three pytest suites passed all 30 cases (16 kinematics, 3 model /
configuration, 11 ROS controller). `colcon test-result` reports 33 tests because it
also includes the three CTest suite wrappers; there were no errors or failures.

Physical tests use the bundled flat world, default geometry and headless server.
Seven motions run for six simulation seconds each, separated by zero commands.
The steady velocities below average the last second. Encoder odometry is compared
against a separate Gazebo ground-truth publisher; that ground truth is never fed
into the controller or encoder estimator.

The two-robot test validates both robots' odometry freshness, separate TF chains
and controller activation, then drives robot1 while robot2 remains uncommanded.

| Motion | Command (vx, vy, wz) | Measured steady motion | Segment odometry error (m / rad) |
|---|---|---|---|
| Forward | 0.30, 0, 0 | 0.300, 0, 0 | 0.0236 / 0.0000 |
| Reverse | -0.30, 0, 0 | -0.300, 0, 0 | 0.0234 / 0.0000 |
| Left | 0, 0.25, 0 | 0, 0.250, 0 | 0.0202 / 0.0114 |
| Right | 0, -0.25, 0 | 0, -0.250, 0 | 0.0193 / 0.0000 |
| Counterclockwise | 0, 0, 0.35 | 0, 0, 0.374 | 0.0180 / 0.1365 |
| Clockwise | 0, 0, -0.35 | 0, 0, -0.374 | 0.0000 / 0.1696 |
| Combined | 0.25, 0.15, 0.30 | 0.256, 0.152, 0.302 | 0.0855 / 0.0617 |

After commands stopped, measured velocities became zero. The other robot's
position and yaw delta remained zero. Both single-robot and two-robot motion
checks passed. The runner cleans up its launch processes after completion.
In the final single-robot run, Gazebo did not exit within launch's five-second
SIGINT grace period and was terminated by launch's SIGTERM escalation after the
motion checks had completed. The temporary test container exited successfully.

Acceptance limits used by the physical smoke test: steady velocity errors below
0.08 m/s in x/y and 0.12 rad/s in yaw; segment encoder pose error below 0.25 m and
0.20 rad; stopped velocities below 0.03 m/s or rad/s. ROS samples arrive
asynchronously, so these are functional regression checks, not precision metrology.

The observed pure-spin rate is about 6.8% above the requested rate in this model;
encoder yaw differs from physical yaw by up to about 0.17 rad during a six-second
segment. Wheel contact and slip are not calibrated to a real chassis. The result
supports functional swerve simulation, not a claim of real-world dynamic accuracy.

GUI rendering, real hardware, sloped/rough terrain and other ROS/Gazebo versions
were not validated. The executable tests in `test/` are provided for reproducing
the checks after moving the package to another workspace.

## URDF split validation (2026-09-27)

After splitting the model into `chassis.urdf.xacro`, `plugins.urdf.xacro` and
`swerve_drive.urdf.xacro`, an isolated package build passed. The three pytest
suites passed all 32 cases; `colcon test-result` reported 35 tests, including
three CTest suite wrappers, with no errors or failures. The additional model
checks render the chassis alone, the complete model and a custom YAML geometry.

The two-robot physical test passed all seven motions, stopped on command
timeout and kept the uncommanded robot stationary. A single-robot test also
passed all those checks in a separate container. Two earlier single-robot runs
exceeded the smoke test's lateral encoder-odometry error threshold during one
of the sideways motions, although their measured steady velocities met the
command targets. A comparison of expanded URDFs found the same model and
plugin fields before and after the split; the root name and XML element order
changed. The intermittent lateral odometry error remains a simulation test
limitation and has not been attributed to the URDF split.
