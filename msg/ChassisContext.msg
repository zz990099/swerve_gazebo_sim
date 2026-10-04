# Current task/obstacles in the controller's local odometry frame.
# Refresh even when unchanged. Never silently restamp stale perception.
std_msgs/Header header
uint64 path_id
uint8 FOLLOW_PATH=0
uint8 GOAL_ONLY=1
uint8 heading_policy
geometry_msgs/Pose2D[<=4096] path
CircleObstacle[<=128] obstacles
