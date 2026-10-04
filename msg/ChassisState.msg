# Current synchronous encoder observation in the local odometry frame.
std_msgs/Header header
string body_frame
geometry_msgs/Pose2D pose
geometry_msgs/Vector3 velocity
float64[4] steering_angles
# Linear rolling m/s, FL FR RL RR (not rad/s).
float64[4] wheel_speeds
uint8 actual_mode
bool confirmed
bool fault
uint8 phase
uint64 request_id
float64 time_in_mode
bool has_accepted_request
uint8 accepted_mode
geometry_msgs/Vector3 accepted_entry_velocity
float64[4] accepted_steering
uint64 session_id
uint64 sequence
uint8 endpoint_fault
uint8 timing_error
uint8 safety_error
# Simulation time at which the next new command is required.
builtin_interfaces/Time next_execute_at
