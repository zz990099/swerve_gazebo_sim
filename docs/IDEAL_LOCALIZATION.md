# Ideal localization for simulation validation

`publish_ground_truth_map_tf:=true` on either version-specific demo/spawn launch
enables a simulation-only localization node. The default is false. This switch
also enables the existing 50 Hz Gazebo ground-truth source and its dedicated ROS
Odometry bridge. The normal generic bridge and wheel controller are unchanged.

```bash
ros2 launch swerve_gazebo_sim demo_gz.launch.py publish_ground_truth_map_tf:=true
```

The optional node publishes only dynamic `map -> odom`. The chassis continues
to publish unmodified wheel `/odom` and `odom -> base_footprint`. `map_frame`
defaults to `map` and denotes the fixed reference of the Gazebo truth source;
no additional world/map transform is broadcast. Namespace/prefix and configured
odom topic/parent are supported. The child must remain the model's
`base_footprint`, so the two observations describe the same physical point.

For every truth timestamp, wheel position is interpolated between bracketing
source-stamped observations (yaw interpolation wraps). The correction is
`T_map_odom = T_truth_map_body * inverse(T_wheel_odom_body)`. It is stamped at
that source time, not at callback arrival. Gaps above 100 ms, invalid/replayed
observations and data older than 200 ms are rejected; no extrapolation occurs.
Clock reversal resets the histories.

This is an ideal localization input, not SLAM. It must be disabled when a real
localization system owns `map -> odom`. Do not simultaneously publish a direct
truth `map -> base_footprint` TF. The wheel odometry retains its natural drift;
the global TF compensates it. Tests using this option measure controller
behavior with ideal external pose observations, not real SLAM accuracy.
