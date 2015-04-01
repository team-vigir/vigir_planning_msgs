uint8 ORIENTATION_FULL=0
uint8 ORIENTATION_AXIS_ONLY=1
uint8 ORIENTATION_IGNORE=2

# The current robot state
moveit_msgs/RobotState current_state

# Waypoints given in frame as specified in header
geometry_msgs/Pose[] waypoints

# Waypoint time stamps [s]
float64[] waypoint_times

# End-effector link name
string[] target_link_names

# How to apply target orientation: full, axis only or ignore
uint8 target_orientation_type

# Specify the sampling rate of the resulting trajectory (in Hz)
float64 trajectory_sample_rate

# Specify whether the planner should check for self-collisions
bool check_self_collisions

# Joints that may be moved
string[] free_joint_names
