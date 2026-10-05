# MPPI ROS planning adapter (0.5)

## Collision geometry and physical model measurement

`config/swerve.yaml` has an optional `safety` mapping. Legacy files without it
use `robot_radius: auto` and `collision_margin: 0.05`. Automatic radius covers
both the chassis box and every steering angle of the wheel collision cylinders:

```
max(hypot(body_length/2, body_width/2),
    hypot(wheelbase/2, track_width/2) + hypot(wheel_radius, wheel_width/2))
```

The bound rounds outwards to centimetres, retaining 0.5 m for the default model.
A numeric `robot_radius` must be finite, positive and no smaller than that bound;
the separately configured margin is finite and nonnegative. Both resolved values
reach the planner and execution validator from the same configuration. A 1.4 m
body automatically uses 0.72 m, so an obstacle already inside that body cannot be
authorized using the old 0.5 m envelope. Restart to change safety parameters.
Directly launched nodes must receive these same resolved parameters; adding new
collision geometry requires updating this bound before using the model.

`ChassisState` adds diagnostic `has_prediction`, source stamp/pose, and endpoint
stamp/pose from the actual installed checked profile. Faults clear the forecast.
These fields do not authorize execution, renew a source timestamp or replace
measured feedback. Rebuild all consumers after this message change.

Both physical MPPI probes now export `model-metrics.json` in their log directory.
Truth and forecasts are buffered during motion; statistics run at stopped
boundaries before rearming and at the final report, outside context publication.
Failed probes also write available partial statistics without suppressing the
original failure; sample sufficiency is explicit in the report.
They compare one-interval model motion with independent Gazebo truth in each
observation's own starting body frame, avoiding accumulated encoder/world frame
offsets. Truth is interpolated only between observations no more than 40 ms apart,
never extrapolated. Reports include matched/moving/unmatched/pending counts and
moving-interval translation/yaw RMS, P95 and maximum error. A latched stop removes
the truncated installed interval and counts it separately as interrupted;
an intentionally cancelled profile is not treated as full-interval model error.
At least five moving
intervals must be observed; missing matches cannot masquerade as zero error.
The probe enforces a bundled-plant regression envelope, not a calibrated hardware
safety tolerance. See the 0.6 acceptance section below.
Measure braking, turning and mode transitions under intended mass, friction and
servo settings before selecting uncertainty margins or claiming model agreement.

For target-host capacity, build core measurement tools and run
`swerve_mppi_integration_budget 200 --budget-ratio 0.6 --strict`.
This fails on any compute timeout or measured pipeline overrun; ordinary CI only
records timing because runner speed is not a portable acceptance requirement.

Build with `SWERVE_BUILD_CHASSIS_CONTROLLER=ON` and the pinned installed standalone
core described in [CHASSIS_INTERFACE.md](CHASSIS_INTERFACE.md). The core repository
remains independent of ROS. The simulation package adds a non-realtime planning
node; the existing ros2_control plugin owns joint execution and revalidates at
the actual physical snapshot.

```bash
ros2 launch swerve_gazebo_sim mppi.launch.py gazebo_version:=gz
# Humble/Fortress: gazebo_version:=ign
```

Launch shares geometry, joint limits, frames and namespace with the execution
controller. The shared `mode_switch_timeout` supplies the confirmation deadline
(5 s in the bundled simulation); other model and planning parameters remain core
0.20.2 defaults. The planner defaults to
60 ms per 100 ms model interval. `compute_budget_ratio` is startup-only, in
(0, 0.7]; a late or absent planning result cancels execution. It cannot be disabled
for live control. These defaults leave a transport margin, not a certified latency
bound. Missed physical deadlines still stop at the independent endpoint.

Planning and protocol callbacks use a separate mutually exclusive callback group
from context and ROS clock reception, with two executor threads. Context forwarding
continues while MPPI computes. Input snapshots are locked only during copies;
malformed context immediately cancels and poisons any concurrent result. Task
identity is checked again before publication. The execution endpoint remains the
independent owner of stopping and physical deadlines.

An eight-entry context history handles independent topic arrival order. Planning
selects the newest original obstacle observation no later than its measured state,
within 150 ms and matching the latest full task. It never retimestamps perception
or revives a replaced task; invalid input clears the history. The execution owner
still checks the newest context at physical application.

The chassis-only stamped Gazebo hardware fixes the upstream position servo gain
to 1.0. The upstream default 0.1 creates roughly ten controller ticks of steering
lag while wheel velocity follows promptly, violating the affine joint model in
curves. Both supported versions use `gain * update_rate * position_error` as the
steering velocity. With 200 Hz physics / 100 Hz control the bundled gain leaves
about one controller tick of lag. Physical regression must validate this plant;
the core's rolling-vector consistency tolerance remains 0.02 m/s. This simulator
servo setting is not a hardware calibration.

## Input, output and ownership

All names below are relative to the robot namespace, volatile depth one.
Commands remain reliable. Context and state snapshots use best effort; subscribe
to state with best-effort QoS. Snapshot loss does not renew timestamps or disable
any watchdog.

| Interface | Type | Contract |
| --- | --- | --- |
| `mppi_planner/context` | `ChassisContext` | Full stable path, path ID, heading policy and current circular obstacles; externally refreshed |
| `chassis_controller/state` | `ChassisState` | Original physics-stamped encoder pose, body/joint motion and complete accepted mode receipt |
| `chassis_controller/command` | `ChassisCommand` | MPPI body velocity or explicit request, original task/source time, session, sequence and next execution time |
| `chassis_controller/context` | `ChassisContext` | Independently checked execution context, forwarded without timestamp changes |
| `mppi_planner/state` | `PlannerState` | Protocol phase, last planned path ID, navigation diagnostics and compute time |
| `mppi_planner/arm` | `std_srvs/Trigger` | Send one new-session stopped arm; await Running feedback |
| `mppi_planner/stop` | `std_srvs/Trigger` | Cancel and latch; a new explicit arm is required |

Context must already be in the controller's encoder odometry frame; frame IDs
are checked exactly, without TF conversion. Keep the full path unchanged during
a task, refreshing its original geometry and current obstacles at least every
0.1 simulation seconds. The maximum source age is 0.15 s. Changing path ID or
geometry is a new task. The task owner should stop and rearm when replacing a
task, because a previously queued command from the old task will be rejected by
the independent task check. Empty obstacles explicitly declare free space.
This adapter does not supply perception or obstacle extraction.

State wheel speeds are already linear m/s; they are copied unchanged. Accepted
mechanical steering and original entry velocity are preserved exactly. Source
measurements are never extrapolated or stamped as a future execution state.
Each newly advertised boundary produces at most one compute call. Joint commands
are generated only by the existing TimedExecutor/ProfileRunner execution path.
Do not run another chassis command/context publisher alongside this node.

## Startup, completion and recovery

1. Publish fresh context for at least 0.1 simulation seconds. Wait for fresh
   chassis feedback and independently verify physical stopping.
2. Call `ros2 service call /mppi_planner/arm std_srvs/srv/Trigger '{}'`.
   A successful response means the arm was sent, not that motion is authorized.
   Await `PlannerState.phase=2` (Running) and healthy matching chassis session.
3. Keep refreshing context. `goal_reached` requires measured position/yaw
   tolerances and settling; it continues streaming zero commands in the final
   mode. Check the diagnostic's path ID against the current task.
4. Use `/mppi_planner/stop` to cancel. Input silence, malformed input, controller
   faults or planning timeout also stop and latch. They never auto-rearm.
5. Verify physical stopping, restore fresh inputs and explicitly call arm again.
   Sessions increase across node restarts using epoch microseconds and observed
   endpoint history. Request high-water marks survive planner resets and are
   recovered from endpoint feedback; process restart does not reuse accepted IDs.

The planner's independent 150 ms wall watchdog checks state and context silence,
including simulation pauses. Execution has its own watchdog even if the planner
is blocked or exits. Restart the process to change model/frame parameters.

Core 0.20.2 is required: a GoalOnly task starting in Spin enters checked terminal
translation once the ordered endpoint is eligible. This prevents switch costs
from leaving short translations indefinitely in Spin. Mode dwell, measured
stopping, mechanical confirmation and full switch/stop trajectory checks remain.

## Validation and limits

```bash
python3 test/run_simulation.py --gazebo-version gz --mppi --logs artifacts/mppi
python3 test/run_simulation.py --gazebo-version gz --mppi-paths --logs artifacts/mppi-paths
```

The offline bridge regression uses the real MPPI Controller, TimedExecutor and
ProfileRunner with an ideal encoder plant, scheduling each plan one interval
ahead. The Gazebo probe runs the real ROS planning process, checks straight,
lateral Crab and Spin goal completion against separate Gazebo physical truth,
then verifies context loss and explicit stopped recovery. Existing interface,
motion and external endpoint regressions remain enabled in both ROS/Gazebo CI
variants.

Physical mode assertions require observing the expected mode while moving.
Terminal settling may legitimately switch from Crab or DualAckermann to Spin
to correct residual yaw; completion still requires physical stopping and the
unchanged independent position/yaw limits.

The extended probe runs reverse/forward, a 21-point radius-2 m arc through four
static cylindrical collision bodies, and translation with GoalOnly final yaw.
The generated SDF and context share one obstacle fixture. It checks independent
Gazebo truth at intervals no greater than 150 ms, including swept center-chord
clearance with the core's 0.5 m body radius and 0.05 m margin. Position/yaw goal
limits are 0.12 m/rad. A malformed obstacle radius while moving must cancel and
latch; restoring valid input cannot auto-rearm. A new stopped session recovers.
`world:=/absolute/path/world.sdf` also forwards a custom world to MPPI launch.

These are bounded deterministic scenarios. They do not certify
obstacle perception, arbitrary paths, physical collision/braking margins or slip
robustness. Planning and feedback use encoder odometry. External localization,
Nav2 actions and TF conversion are subsequent integration work.

## 0.6 acceptance and diagnostics

Requires core 0.21.0. Healthy ChassisState includes context simulation age, command
source/adapter-receipt ages at application and schedule slack at receipt.
PlannerState includes feedback age, selected context age and post-compute schedule
margin. -1 indicates unavailable diagnostics. These fields never renew timestamps
or authorize motion. A receipt age excludes DDS delay before the adapter callback;
source age includes the observation-to-application interval.

The physical probes publish their synthetic static task/circles immediately on
each observed ChassisState, preserving its exact source stamp. They do not analyze
truth inside this publication loop. Real perception must retain its own original
measurement stamp; publishing a cached obstacle list with a new stamp is invalid.

Each probe must have at least 20 moving intervals; at least five for each required
mode and braking. Straight/Crab/Spin probes require modes 0/1/2, corridor probes
0/1. At least 90% of completed predictions must match bounded independent truth;
missing installed-profile sequence numbers count against coverage.
Explicitly interrupted intervals are reported separately; end-of-probe pending
intervals remain visible. Global and each covered group's moving error must meet:

| Metric | P95 limit | Maximum limit |
| --- | --- | --- |
| One-profile relative translation | 0.005 m | 0.010 m |
| One-profile relative yaw | 0.015 rad | 0.025 rad |

These regression limits deliberately exceed the recorded nominal 0.5.0 maxima
(about 2.3 mm and 0.010 rad) while detecting material model degradation. They do
not certify slip uncertainty, sensor accuracy or collision margins. Full-path
truth goal position/yaw gates and physical corridor clearance remain separate.
Installed forecasts also carry source/end velocity to classify braking; group
coverage prevents stationary holds from diluting motion error.

Both ROS/Gazebo CI families run three independently isolated nominal MPPI probes,
three explicit-chassis probes, and a 25% body-mass payload MPPI probe. The payload
changes only the plant's mass/inertia, not controller limits. No failed physical
run is retried inside the test script. Runtime regressions inject queue/observation
delay, preserve source stamps and ensure fresh packets cannot hide stale context.

Tire-friction/slip, servo-lag and larger plant changes still need identification
and separate scenario acceptance; this bounded payload test does not cover them.
The target CPU must pass the core's 60 ms strict workload gates under expected
load, and repeated ROS runs must retain schedule headroom and fresh context.

## 0.6.1 snapshot transport

A repeated Jazzy run at 0.6.0 exposed a common feedback/context gap: both the
planner and probe retained an old state while physical time advanced, causing
the unchanged 150 ms context gate to stop execution. Reliable high-rate snapshot
publication can create backpressure and prevents timely newest-frame delivery.
State/context transport now uses best effort, volatile KeepLast(1), following
ROS sensor-data semantics. Low-rate command/session authorization remains reliable.
Clients must request best effort for ChassisState and PlannerState; a reliable
state subscriber is incompatible with a best-effort publisher.

The message reports cumulative realtime publisher skips. Probe metrics report
wall/simulation feedback gaps. Repeated nominal runs also attach a deliberately
slow diagnostic observer (200 ms per callback); it must not stall the control
stream. This addresses snapshot backpressure; it does not guarantee scheduling
on an arbitrarily overloaded host. All source stamps, fault latches, model error
thresholds, coverage requirements and independent execution watchdogs are unchanged.
