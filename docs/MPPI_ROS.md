# MPPI ROS planning adapter (0.4)

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
controller. Other execution model and planning parameters remain core 0.20
defaults. Only the whole-compute wall budget differs: the planner defaults to
60 ms per 100 ms model interval. `compute_budget_ratio` is startup-only, in
(0, 0.7]; a late or absent planning result cancels execution. It cannot be disabled
for live control. These defaults leave a transport margin, not a certified latency
bound. Missed physical deadlines still stop at the independent endpoint.

## Input, output and ownership

All names below are relative to the robot namespace, reliable/volatile depth one.

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

## Validation and limits

```bash
python3 test/run_simulation.py --gazebo-version gz --mppi --logs artifacts/mppi
```

The offline bridge regression uses the real MPPI Controller, TimedExecutor and
ProfileRunner with an ideal encoder plant, scheduling each plan one interval
ahead. The Gazebo probe runs the real ROS planning process, checks straight,
lateral Crab and Spin goal completion against separate Gazebo physical truth,
then verifies context loss and explicit stopped recovery. Existing interface,
motion and external endpoint regressions remain enabled in both ROS/Gazebo CI
variants.

These are initial deterministic scenarios in an empty world. They do not certify
obstacle perception, arbitrary paths, physical collision/braking margins or slip
robustness. Planning and feedback use encoder odometry. External localization,
Nav2 actions and TF conversion are subsequent integration work.
