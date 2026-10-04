# Body-frame command. Mode is explicit, never inferred from velocity.
uint8 DUAL_ACKERMANN=0
uint8 SPIN=1
uint8 CRAB=2
std_msgs/Header header
uint64 session_id
uint64 sequence
builtin_interfaces/Time source_stamp
builtin_interfaces/Time execute_at
builtin_interfaces/Time valid_until
# Explicit stopped startup/recovery into a strictly newer session.
bool arm
# false cancels authorization and latches a stop; zero velocity is a normal stop.
bool authorized
uint8 mode
geometry_msgs/Vector3 velocity
bool has_mode_request
uint64 request_id
uint8 requested_mode
# x=vx [m/s], y=vy [m/s], z=wz [rad/s]. Alignment intent, not drive permission.
geometry_msgs/Vector3 entry_velocity
# Exact source task used by planning. Independently compared with latest context.
uint64 path_id
uint8 heading_policy
geometry_msgs/Pose2D[<=4096] source_path
