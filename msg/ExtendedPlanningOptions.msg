uint8 TYPE_FREE_MOTION=1
uint8 TYPE_CARTESIAN_WAYPOINTS=2
uint8 TYPE_CIRCULAR_MOTION=3

uint8 ORIENTATION_FULL=0
uint8 ORIENTATION_AXIS_ONLY=1
uint8 ORIENTATION_IGNORE=2

# Specify the desired motion type
uint8 target_motion_type

# Specify frame in which target poses below are specified
string target_frame

# Specify an identifier for each waypoint
string[] target_pose_names

# Specify the desired waypoints. The following semantics apply:
# TYPE_FREE_MOTION: Single target pose
# TYPE_CARTESIAN_WAYPOINTS: Waypoints in arbitrary frame
# TYPE_CIRCULAR_MOTION: Rotation center

geometry_msgs/Pose[] target_poses

# Specify the times when the target poses should be reached
# (in seconds, relative to the start of the trajectory)
float64[] target_pose_times

# Specify the target link for each target pose
string[] target_link_names

# Specify how the orientation is to be interpreted at each
# waypoint - ignore, only use axis or full orientation
uint8 target_orientation_type

# Specify the sampling rate of the resulting trajectory (in Hz)
float64 trajectory_sample_rate

# Specify whether the planner should check for self-collisions
bool check_self_collisions

# Specify if robot should continuously replan motion plan
bool continuous_replanning

# Circular motion settings below
#----------------------------------------------

# Used for specifying circular motion rotation angle
# Can be positive or negative
float32 rotation_angle

# Keep endeffector orientation if true (do not rotate around rotation_center_pose)
bool keep_endeffector_orientation
