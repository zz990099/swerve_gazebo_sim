# Synchronous physics-stamped chassis feedback. Wheel order FL, FR, RL, RR.
uint8 DUAL_ACKERMANN=0
uint8 SPIN=1
uint8 CRAB=2
uint8 FAULT=0
uint8 BRAKING=1
uint8 ALIGNING=2
uint8 READY=3
std_msgs/Header header
uint64 session_id
uint64 sequence
uint64 request_id
uint8 actual_mode
uint8 requested_mode
uint8 phase
bool confirmed
# 0 none, 1 disarmed, 2 clock, 3 feedback, 4 command, 5 timeout,
# 6 transition timeout, 7 command interface write failure.
uint8 fault
geometry_msgs/Twist velocity
float64[4] steering_angles
# Raw drive joint angular velocities, rad/s.
float64[4] wheel_speeds
geometry_msgs/Twist accepted_entry_velocity
float64[4] accepted_steering
