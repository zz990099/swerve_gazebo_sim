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
# Diagnostic forecast of the installed, checked profile; never command authority.
# Compare relative motion from prediction_source_pose against independent truth.
bool has_prediction
builtin_interfaces/Time prediction_source_stamp
geometry_msgs/Pose2D prediction_source_pose
builtin_interfaces/Time prediction_stamp
geometry_msgs/Pose2D predicted_pose

# Diagnostics only; source/receipt ages are measured at application; slack at receipt.
# Context age uses simulation time; receipt age uses steady wall time.
float64 context_age_seconds
float64 command_receipt_age_seconds
float64 command_source_age_seconds
float64 command_schedule_slack_seconds
geometry_msgs/Vector3 prediction_source_velocity
geometry_msgs/Vector3 predicted_velocity
