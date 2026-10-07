# Body velocity and explicit mode/alignment request. No planner/task data.
uint8 DUAL_ACKERMANN=0
uint8 SPIN=1
uint8 CRAB=2
std_msgs/Header header
uint8 mode
# Body linear x/y in m/s and angular z in rad/s. Other components must be zero.
geometry_msgs/Twist velocity
# New request: increase ID and use zero velocity. Repeat unchanged entry while waiting.
# Ordinary drive/hold retains the last request ID and entry. Startup ID is zero.
uint64 request_id
# Frozen alignment intent, never drive permission. Zero selects canonical geometry.
geometry_msgs/Twist entry_velocity
