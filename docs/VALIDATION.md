# Validation record

This is a chronological record. Earlier combined lateral/yaw results describe
the former unrestricted command interface, not the current three-mode controller.
The current supported motions are documented in README.md and tested by the
three-mode suite below. Historical counts and measurements are not claims about
later revisions.

Current tests read ground truth directly from Gazebo Transport JSON without a
ROS odometry bridge. Runtime odometry remains a wheel/steering kinematic estimate.
Launch uses one generic bridge and exposes independent Odometry/TF output settings.
The dated records below describe the configuration used at that time.

## Feedback ordering and clock epoch regression (2026-10-02, 0.2.2)

A delayed JointState within the feedback freshness window previously reset pose
when its header stamp was older than the last accepted observation, even if ROS
time had not rolled back. The callback now rejects older/repeated observations
using original integer nanoseconds. It integrates elapsed integer nanoseconds, so
adjacent stamps remain distinguishable at large epochs. Rejected observations do
not change measured state, cancel commands or publish odometry/TF.

All runtime callbacks observe the ROS clock independently of message headers.
A real clock rollback clears command, feedback and odometry state in either
control mode and resets the internal supervisor when present. The drive timer
does not clear new-epoch feedback if another callback already observed the reset.
Regression cases cover older/repeated stamps, one-nanosecond intervals at a large
epoch, clock resets first observed by feedback/status/the drive timer, and a reset
with malformed feedback. Real ROS callback tests run in both distribution CI jobs.

## Interrupted transition regression (2026-09-30)

The previous controller could resume differential drive before steering returned
home when a pending crab/spin transition was cancelled back to differential.
The fix clears the old active mode when a transition begins. Regression cases
cover both interrupted returns and all six directed changes among the three modes.
The kinematics suite passed 26 cases locally; isolated checks of the actual timer
methods passed the six direct transitions and both interrupted returns. Full ROS
controller and physical tests require a sourced ROS/Gazebo environment and are
not implied by these isolated checks.

## Transition supervisor validation (2026-09-30)

The ROS-independent supervisor adds hysteresis, request dwell, measured wheel-stop
confirmation, stable steering alignment, diagnostic state and a latched transition
deadline. Local kinematics, model/configuration and supervisor suites passed 61
cases. The supervisor tests use explicit simulation timestamps and cover all six
direct mode changes, interrupted returns, third-mode retargets, watchdog recovery,
crab direction changes, noisy commands, timeout/reset and backwards clock resets.
Additional ROS adapter cases exercise measured braking and DiagnosticArray output.
Those adapter cases and physical motions must run in the ROS CI matrix.

## Continuous validation matrix (2026-09-30)

`.github/workflows/ci.yaml` defines Humble/Fortress and Jazzy/Harmonic jobs using
the Docker official `ros` base images. Each job installs public dependencies with
rosdep, builds this package in an isolated workspace, runs all six pytest suites,
then runs single-robot physical tests. Physical tests cover diagonal
crab, all six directed mode switches without intermediate stop commands, interrupted
returns and timeout/home behavior. The runner has a bounded 300-second deadline and cleans up its
launch processes. Actions uploads build, test-result and launch logs on failure
as well as success. The workflow result for a commit is the evidence for that
commit; adding the workflow alone does not assert that its jobs passed.

The first CI run passed 97 pytest cases (102 including CTest wrappers) and both
single-robot physical runs. Both subsequent multi-robot runs detected another
world's clock publisher and failed the final interrupted-return velocity check.
The runner now gives each invocation a unique Gazebo transport partition and
tracks simulator descendants before launch can exit, then terminates any survivors.
A regression test covers a detached child that ignores SIGTERM after its launch
parent exits. Local suites including runner tests passed 63 cases. This correction
keeps the physical acceptance limits unchanged.

The current acceptance scope is single-robot simulation. The multi-robot CI step
has been removed at the user's request; it is not a required check. The two
ROS/Gazebo jobs still exercise their fixed-version demo, physical motion modes,
direct switches, interrupted returns and timeout stopping.

## Original validation (2026-09-25)

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

## Gazebo family and style validation (2026-09-28)

The launch, controller and test Python files were formatted with Black and
checked with Ruff. `package.xml`, all Xacro files and both bundled SDF worlds
passed XML parsing. An isolated Humble workspace build passed all 37 reported
tests with no errors or failures.

The Fortress two-robot physical test passed all seven motion segments, command
timeout stopping, separate TF and odometry chains, and the stationary-peer
check. This run used the new `clock_bridge_ign.yaml` and
`robot_bridge_ign.yaml` files, so it also verifies the configuration-file bridge
path and relative namespaced ground-truth topic. A final single-robot run after
the controller readability changes passed the same motion and stopping checks.

The Harmonic model, `gz.msgs` bridge configuration and `gz-sim` world were
validated by Xacro expansion, YAML assertions and XML parsing. A Harmonic
runtime was not installed in the validation container, so the `gz` family was
not physically simulated in this record.

## Steering interlock validation (2026-09-28)

An isolated Humble workspace build passed. The kinematics suite passed 24 cases,
the model suite passed 7 cases and the controller suite passed 16 cases.
`colcon test-result` reported 50 tests, including the three CTest suite wrappers,
with no errors or failures. The controller cases verify spin and crab alignment
gates and the differential entry-home gate. They also verify that explicit stop,
command timeout and differential-mode entry return steering targets to zero.

The Fortress physical test passed forward, reverse, lateral, spin and combined
swerve motion. After command timeout, the largest measured chassis velocity was
0.00023 rad/s and the largest steering-joint offset was 0.0000046 rad. This run
therefore verified both chassis stopping and steering return-to-home behavior in
simulation. Black formatting, Ruff checks and Git whitespace checks also passed.

## Three-mode control validation (2026-09-28)

The controller now exposes differential, spin and crab as its three motion modes.
Differential mode accepts `vx + wz`, waits for zero steering on entry and does not
reapply the steering interlock while active. Unit tests inject steering error during
an active differential maneuver and verify that drive commands continue. Its active
kinematics use opposite front/rear steering targets and per-module speeds for
double-Ackermann motion. Combined lateral translation and yaw is rejected with a
stop command.

The final isolated build reported 54 tests with no failures. In the Fortress
physical test, a `(0.25, 0.0, 0.2)` differential command produced approximately
`(0.254, 0.000, 0.203)`, with 0.036 m planar odometry error and 0.024 rad yaw
error over the segment. The same run passed forward, reverse, crab in both
directions, spin in both directions, timeout stopping and steering return-to-home.

## CPU usage validation (2026-09-28)

The original bundled world used a 0.001 s step, producing 1000 physics and `/clock`
updates per simulated second. In an isolated headless Fortress run, Gazebo used
about 48% of one CPU core, the Python controller 46% and the clock bridge 10%.
Changing the step to 0.005 s reduced those measurements to approximately 19%, 23%
and 4%, while the measured real-time factor remained between 0.97 and 0.99.

The 200 Hz world passed the complete physical smoke test: forward, reverse, both
crab directions, both spin directions, double-Ackermann differential turning,
timeout stopping and steering return-to-home. A container without a GPU render
device still uses CPU software rendering when the GUI is enabled; use
`headless:=true` or provide hardware OpenGL access to avoid that separate cost.
