uint8 TYPE_FREE_MOTION=1
uint8 TYPE_CARTESIAN_WAYPOINTS=2
uint8 TYPE_CIRCULAR_MOTION=3

# Specify the desired motion type
uint8 target_motion_type

# Specify frame in which target poses below are specified
string target_frame

# Specify the desired waypoints.
# If this pose array is empty, the Action is interpreted
# in the standard MoveIt! way without consideration
# of extended planning options.
#
# The following semantics apply:
# TYPE_FREE_MOTION: Single target pose
# TYPE_CARTESIAN_WAYPOINTS: Waypoints in arbitrary frame
# TYPE_CIRCULAR_MOTION: Rotation center
geometry_msgs/Pose[] target_poses

# Specify frame in which reference point below is specified
string reference_point_frame

# Specify the desired reference point
# If this is the Identity, the reference point will be
# interpretaded as the wrist, Otherwise, the poses for the
# wrist are going to be transformed to this reference point.
geometry_msgs/Pose reference_point

# Specify if robot should continuously replan motion plan
bool continuous_replanning

# Specify if collisions should be avoided
bool avoid_collisions

# Determines if incomplete cartesian plans should be executed
# (Happens if target moves out of reach). Defaults to false.
bool execute_incomplete_cartesian_plans

# Drake-specific settings below
#----------------------------------------------

uint8 ORIENTATION_FULL=0
uint8 ORIENTATION_AXIS_ONLY=1
uint8 ORIENTATION_IGNORE=2

# Specify an identifier for each waypoint
string[] target_pose_names

# Specify how the orientation is to be interpreted at each
# waypoint - ignore, only use axis or full orientation
uint8 target_orientation_type

# Specify the sampling rate of the resulting trajectory (in Hz)
float64 trajectory_sample_rate

# Specify whether the planner should check for self-collisions
bool check_self_collisions

# Specify the times when the target poses should be reached
# (in seconds, relative to the start of the trajectory)
float64[] target_pose_times

# Specify the target link for each target pose
string[] target_link_names

# Specify axis for each target link (for ORIENTATION_AXIS_ONLY)
geometry_msgs/Point[] target_link_axis


# Circular motion settings below
#----------------------------------------------

# Used for specifying circular motion rotation angle
# Can be positive or negative
float32 rotation_angle

# Keep endeffector orientation if true (do not rotate around rotation_center_pose)
bool keep_endeffector_orientation
