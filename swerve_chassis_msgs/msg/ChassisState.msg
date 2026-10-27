# Version 1. Wheel order FL FR RL RR, angles rad, drive speeds rad/s.
# Header is publication time; feedback_stamp is the actual encoder sample time.
std_msgs/Header header
builtin_interfaces/Time feedback_stamp
builtin_interfaces/Time command_stamp
uint64 command_sequence
float64[4] steering_angles
float64[4] wheel_speeds
geometry_msgs/Twist velocity
geometry_msgs/Twist requested_velocity
geometry_msgs/Twist executable_velocity
# Complete Generator State; pass target arrays and generator_valid into core.
float64[4] target_steering
float64[4] target_wheel_speeds
bool generator_valid
# TRACKING=0 LIMITED=1 BRAKING=2 PRESTEERING=3 STOPPED=4
uint8 execution
# NONE=0 COMMAND=1 FEEDBACK=2 CLOCK=3
uint8 fault
bool command_fresh
bool feedback_valid
float64 consistency_residual
# Startup-only shared physical configuration (order specified in interface docs).
float64[3] geometry
float64[10] limits
