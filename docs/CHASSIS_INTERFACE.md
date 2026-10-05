# Explicit chassis velocity and mode interface (0.3)

`ChassisController` receives body velocity and explicit mode requests. It owns
all eight ros2_control command interfaces, replacing either the default Twist
controller or external joint endpoint. It never infers mode from velocity.
It reuses installed MPPI 0.20 `TimedExecutor`, `ProfileRunner` and the existing
`EndpointGuard`. Stochastic planning remains outside the controller manager.
The companion `mppi.launch.py` starts the implemented MPPI planning node; Nav2
lifecycle/TF integration remains separate. Shared `safety` configuration now
propagates the physical collision bound and margin to both planning and execution;
see [MPPI_ROS.md](MPPI_ROS.md) for geometry and prediction diagnostics.

## Build and start

The default build still supports independent Twist/external-joint examples.
To enable the new interface, install the core first:

```bash
git clone https://github.com/zz990099/swerve_mppi.git
cd swerve_mppi
git checkout b1e0057b09fb8f9ca08a9e940386680d55a91469
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DSWERVE_MPPI_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX="$HOME/swerve-core"
cmake --build build --parallel 2
cmake --install build
# In the ROS workspace containing swerve_gazebo_sim:
colcon build --packages-select swerve_gazebo_sim --cmake-args \
  -DSWERVE_BUILD_CHASSIS_CONTROLLER=ON -DCMAKE_PREFIX_PATH="$HOME/swerve-core"
source install/setup.bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py chassis_control:=true
# Humble/Fortress: use demo_ign.launch.py.
```

`chassis_control` and `external_joint_control` are mutually exclusive. Manager
rate is 100 Hz; model intervals are 0.1 simulation seconds. Geometry and
wheel/steering limits and `mode_switch_timeout` come from `swerve.yaml`; other execution parameters are
core 0.20.2 defaults. The planning owner must use the same configuration.
Configuration is startup-only; restart the process to change it. Generated
messages and existing Python modules share the installed package namespace.

## Topics and units

All topics are relative to the robot namespace; reliable, volatile, depth-one QoS.

| Topic | Type | Purpose |
| --- | --- | --- |
| `chassis_controller/command` | `swerve_gazebo_sim/msg/ChassisCommand` | Velocity, optional mode request, source task and timing envelope |
| `chassis_controller/context` | `swerve_gazebo_sim/msg/ChassisContext` | Independently refreshed current task and circular obstacles |
| `chassis_controller/state` | `swerve_gazebo_sim/msg/ChassisState` | Encoder state, mode receipt/confirmation, fault and next execution time |

Mode numbers: **0 DualAckermann, 1 Spin, 2 Crab**. Vector3 encodes
`x=vx [m/s], y=vy [m/s], z=wz [rad/s]` in `body_frame` (normally
`<prefix>base_footprint`). Command frame ID must match exactly. Context and state
pose use `odom_frame` (normally `<prefix>odom`). Heading policy is 0 FollowPath
or 1 GoalOnly. State wheel speeds are **linear rolling m/s**, steering is rad,
in FL, FR, RL, RR order. Do not convert returned wheel speeds a second time.

| Command | Meaning |
| --- | --- |
| `mode=2`, velocity `(0.3,0,0)`, no request | Forward Crab, never reclassified as Ackermann |
| Authorized zero velocity, no request | Controlled braking/holding, retaining mode and steering |
| Zero velocity, request ID, target mode and entry velocity | Brake/align/confirm; no permission to drive |
| `authorized=false` | Immediate cancellation, latched fault and zero wheel commands |
| Missing next command or stale context | Latched stop; no reuse of an old command |

For requests, `mode` and `requested_mode` must agree. Entry velocity specifies
alignment direction/curvature, not drive permission. Retries use new envelope
sequences but identical ID/payload; changed/replayed requests fault. Await matching
request ID, actual mode and confirmed state before driving. Zero does not cancel
a committed transition or clear a fault.

State carries the accepted entry velocity and frozen mechanical steering positions.
Preserve them in MPPI `VehicleState.accepted_mode_request`, using `request_id` as
its ID. The receipt persists after confirmation; never reconstruct it from moving
encoders. Phase/timing/safety enums use core 0.20 ordering. EndpointFault uses the
codes in EXTERNAL_ENDPOINT.md. `fault=true` always withholds drive authorization.

## Timing and execution

Refresh context at least every 0.1 s; maximum source age is 0.15 s. Empty obstacles
explicitly assert free space; missing context does not. Paths are bounded to 4096
points and obstacles to 128, without truncation. Each command's source path/ID/
heading policy must be the original planning task, checked independently against
latest context.

After arm, state advertises `next_execute_at`. Send one fresh command **ahead of**
each boundary, with that execute_at and short valid_until (e.g. boundary + 0.025 s).
Keep the original planning source_stamp and actual issuance header stamp; the
source-to-valid-until span must be <= 0.15 s. Do not repeatedly publish the same
sequence. A newer ordinary command can replace a queued one before acceptance.
Cancellation/malformed callbacks cannot be hidden by later valid input between
hardware updates.

The Gazebo `PhysicsClock` model plugin records `UpdateInfo.simTime` directly in
the entity-component manager. `StampedGazeboSystem` delegates joint physics to
the upstream system and exports that model time during the same synchronous read. The
controller uses that timestamp, rather than the asynchronous controller-manager
ROS clock. It reads joints directly at application, computes encoder FK/local
odometry, and calls TimedExecutor with this current snapshot. It does not restamp
transported ROS observations. Only admitted profiles are sampled at 100 Hz and
passed through EndpointGuard. Missed model boundaries, profile/context expiry,
clock rollback and wall watchdogs latch a stop. Pausing physics does not pause
wall watchdogs. Task/collision rejection uses core's separately checked stop.

Use `state.pose` consistently as the initial planning frame. Python observer odom/TF
remains available for visualization but is independently integrated, not an exact
substitute for this snapshot. Both are encoder odometry, not physical ground truth.
External localization, TF transforms and physical-slip uncertainty are not modeled.
Bounded execution admission allocates working data; this is a simulation plugin,
not a claim of allocation-free hard-real-time hardware control.

## Startup and recovery

1. Wait for fresh state and independently verify physical stopping. Initial mode
   requires canonical measured steering (normally zero-steering Ackermann/Crab).
2. Start refreshing context before sending the arm command; separate DDS topics
   do not guarantee arrival order. Publish one authorized zero command with arm=true, no request,
   and strictly newer nonzero session (< 2^53). Use current source/issuance times,
   execution no later than receipt and a short unexpired validity window.
3. Await matching healthy state and stream at advertised boundaries. Do not repeat
   the arm packet. Reliable transport is used; a failed arm needs a newer session.
4. On fault, verify stopping and recover with a newer session. Recovery retains
   last actual mode and measured steering, including noncanonical stopped alignment;
   arm cannot switch modes. Explicit requests perform subsequent alignment.

History survives lifecycle deactivate/cleanup/reactivate in one plugin instance.
Sender sessions must increase across sender restarts. Simulator restart creates a
new endpoint lifetime; drain old traffic. Reset the external MPPI Controller on
recovery, preserving returned request-ID high-water marks. No other joint writer
may run alongside this controller.

## Validation

`chassis_runtime_tests` exercises actual core execution: explicit mode, zero,
measured confirmation/receipt, mutation, timeouts, task mismatch, cancel and recovery.
`chassis_interface_tests` loads the real plugin with loaned interfaces and typed
messages over DDS, including a delayed ROS clock with current physics time.
Existing default/external tests remain enabled.

```bash
python3 test/run_simulation.py --gazebo-version gz --chassis --logs artifacts/chassis
```

The physical probe checks Crab forward, Spin entry, retained steering/mode on zero,
independent Gazebo motion, publisher silence and recovery. It does not certify
MPPI path tracking, collision margins, slip or braking bounds; these remain
subsequent planning integration and physical calibration work.
