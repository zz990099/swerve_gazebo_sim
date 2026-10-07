# Validation

ROS-independent production-runtime, kinematics, model and truth-decoding tests:

```bash
python3 -m pip install pytest pyyaml xacro
PYTHONPATH=. python3 -m pytest -q test/test_chassis.py test/test_kinematics.py test/test_model.py test/test_truth.py
```

They cover all six directed mode changes, measured stop before alignment, frozen
receipts, same-mode realignment, target limits, reverse motion, zero hold, malformed
commands, command loss, paused clocks, clock rollback, feedback faults and explicit
recovery. Runtime tests model perfect actuator following; physical behavior is
checked separately.

CI builds this repository alone on Humble/Fortress and Jazzy/Harmonic. It tests
Python execution/odometry, ROS message adaptation, launch, strict configuration
and Xacro. Physical acceptance runs the official hardware and forward controllers,
performs all six mode changes, checks independent Gazebo truth translation/rotation,
verifies zero hold, publisher-loss stopping and recovery, then kills the Python
chassis process and checks that the standalone demo exits.

```bash
colcon test --packages-select swerve_gazebo_sim --return-code-on-test-failure
colcon test-result --verbose
python3 src/swerve_gazebo_sim/test/run_simulation.py --gazebo-version gz --logs artifacts/chassis
```

Use `--gazebo-version ign` on Humble/Fortress. Consult the workflow for the exact
commit. Acceptance checks basic physical operation, not calibrated slip, obstacle
navigation, hard real-time timing or external-world process supervision.
