# Body-frame velocity and explicit mode. No navigation task data.
uint8 DUAL_ACKERMANN=0
uint8 SPIN=1
uint8 CRAB=2
std_msgs/Header header
# Strictly newer session for reset; strictly increasing sequence within a session.
uint64 session_id
uint64 sequence
# Reset requires zero velocity/entry, request_id=0 and measured stopped wheels.
bool reset
uint8 mode
# x/y linear m/s, z angular rad/s; other Twist components must be zero.
geometry_msgs/Twist velocity
# Increase for a new mode/alignment request, with zero velocity. Retries retain
# mode, request_id and entry_velocity; only sequence and timestamp advance.
uint64 request_id
# Alignment geometry only, never permission to drive. Zero selects canonical entry.
geometry_msgs/Twist entry_velocity
