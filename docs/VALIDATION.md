# Validation

Portable production-runtime tests run without ROS:

```bash
cmake -S . -B build-independent -DSWERVE_STANDALONE_TESTS=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build-independent
ctest --test-dir build-independent --output-on-failure
```

They cover all six directed mode changes, measured stop before alignment, frozen
mode-entry receipts, same-mode realignment, velocity and actuator limits, zero
hold, malformed/replayed/expired commands, clock changes, feedback faults and
explicit stopped recovery. These tests model perfect actuator following; they
cannot establish physical tracking.

ROS CI builds this repository alone on Humble/Fortress and Jazzy/Harmonic. It
runs kinematics, Xacro, strict configuration, launch, truth-decoding and odometry
unit tests. The physical test loads the actual controller plugin, performs all
six directed mode changes, checks Gazebo truth translation/rotation, verifies
zero hold and publisher-loss stopping. It uses no planner or path fixtures.

```bash
colcon test --packages-select swerve_gazebo_sim --return-code-on-test-failure
colcon test-result --verbose
python3 src/swerve_gazebo_sim/test/run_simulation.py --gazebo-version gz --logs artifacts/chassis
```

Consult the workflow result for the exact commit under review. Historical tests
of removed interfaces do not validate this interface. Physical acceptance is a
bounded smoke test, not a slip calibration, obstacle-navigation benchmark or
proof of real-hardware safety.
