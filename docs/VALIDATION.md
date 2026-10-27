# Validation and migration baseline

Old main baseline: `9358c96884d7fa4866cea7c053daa7a7cec82d8f`.
Its existing CI run 37631105628 passed on 2026-10-07. This preserves the old code
and test recipe; no new old-versus-new physical recordings were obtained locally.

## Portable checks

```bash
cmake -S swerve_chassis_core -B build-core -DCMAKE_BUILD_TYPE=Release
cmake --build build-core -j2
ctest --test-dir build-core --output-on-failure
PYTHONPATH=swerve_gazebo_sim python3 -m pytest -q \
  swerve_gazebo_sim/test/test_model.py swerve_gazebo_sim/test/test_truth.py
```

Core coverage includes 10,000 seeded inverse/forward round trips, coordinated
wheel residual, rate/speed/travel bounds, lateral startup, combined motion,
rotation, reversal, representation boundary, single-wheel zero, zero hold,
state restoration, stalled steering and exact SE(2) integration. Tests throw
on failure in release builds too (they do not rely on disabled `assert`).

## ROS and physical checks

`colcon test` includes a real C++ controller process driven by fake joint states
and a controlled ROS clock. It checks combined velocity, timeout, recovery,
missing feedback, paused time and rollback. This test is separate from physics.

```bash
python3 src/swerve_gazebo_sim/swerve_gazebo_sim/test/run_simulation.py \
  --gazebo-version gz --logs artifacts/chassis
```

The physical probe commands translation, lateral movement, rotation, simultaneous
vx/vy/wz, reversal across steering representations, and a zero-speed wheel.
After settling, independent Gazebo poses must match the requested constant twist
to 0.15 m / 0.10 rad over the observation interval. It also checks zero hold,
command-loss braking, recovery and demo teardown after controller death.
CI runs Humble/Fortress and Jazzy/Harmonic. Humble/Harmonic is not a matrix entry.

Local implementation environment has no ROS or Gazebo. Portable checks can run
here; ROS compilation, node integration and physical acceptance are established
only by CI results, not by the mathematical model. This phase does not establish
MPPI tracking improvement, calibrated actuator response, tire slip bounds,
long-run drift, target-hardware latency, or old/new comparative path metrics.
Those remain integration acceptance work after the MPPI migration.
