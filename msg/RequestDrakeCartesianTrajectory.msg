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

# Axis on end-effector (for ORIENTATION_AXIS_ONLY)
geometry_msgs/Point[] target_link_axis

# How to apply target orientation: full, axis only or ignore
uint8 target_orientation_type

# Specify the sampling rate of the resulting trajectory (in Hz)
float64 trajectory_sample_rate

# Specify whether the planner should check for self-collisions
bool check_self_collisions

# Joints that may be moved
string[] free_joint_names

# Determines if incomplete cartesian plans should be executed
# (Happens if target moves out of reach). Defaults to false.
bool execute_incomplete_cartesian_plans

# Determines if the motion should be along an axis between the points
# or if it is a free motion with only fix endpoints
bool free_motion
