# Phase: 0 Disarmed, 1 Arming, 2 Running, 3 Fault. Completion keeps streaming zero.
std_msgs/Header header
uint8 phase
uint64 session_id
uint64 path_id
uint8 navigation_status
uint8 failure_reason
uint8 control_policy
bool authorized
bool goal_reached
bool stalled
float64 path_progress
float64 remaining_path
float64 cross_track_error
float64 goal_distance
float64 goal_yaw_error
# Duration of the last actual planning call; idle feedback does not overwrite it.
float64 compute_seconds
uint64 evaluated_rollouts
bool budget_exhausted

# Latest observed simulation age; -1 before a compatible planning context.
float64 feedback_age_seconds
float64 planning_context_age_seconds
# Simulation-time margin between post-solve issue and scheduled application.
float64 schedule_margin_seconds
