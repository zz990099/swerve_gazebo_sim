# Last encoder observation, not a synchronous physics-step snapshot. FL, FR, RL, RR.
uint8 DUAL_ACKERMANN=0
uint8 SPIN=1
uint8 CRAB=2
uint8 FAULT=0
uint8 BRAKING=1
uint8 ALIGNING=2
uint8 READY=3
uint8 NO_FAULT=0
uint8 CLOCK_FAULT=1
uint8 FEEDBACK_FAULT=2
uint8 COMMAND_FAULT=3
uint8 COMMAND_TIMEOUT=4
uint8 TRANSITION_TIMEOUT=5
std_msgs/Header header
uint64 request_id
uint8 actual_mode
uint8 requested_mode
uint8 phase
bool confirmed
uint8 fault
geometry_msgs/Twist velocity
float64[4] steering_angles
# Raw drive joint angular velocity, rad/s.
float64[4] wheel_speeds
geometry_msgs/Twist accepted_entry_velocity
float64[4] accepted_steering
