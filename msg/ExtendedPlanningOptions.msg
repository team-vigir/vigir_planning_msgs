uint8 TYPE_FREE_MOTION=1
uint8 TYPE_CARTESIAN_WAYPOINTS=2
uint8 TYPE_CIRCULAR_MOTION=3

# Specify the desired motion type
uint8 target_motion_type

# Specify frame in which target poses below are specified
string target_frame

# Specify the desired waypoints. The following semantics apply:
# TYPE_FREE_MOTION: Single target pose
# TYPE_CARTESIAN_WAYPOINTS: Waypoints in arbitrary frame
# TYPE_CIRCULAR_MOTION: Rotation center

geometry_msgs/Pose[] target_poses

# Specify if robot should continuously replan motion plan
bool continuous_replanning

# Specify if collisions should be avoided
bool avoid_collisions

# Determines if incomplete cartesian plans should be executed
# (Happens if target moves out of reach)
bool execute_incomplete_cartesian_plans

# Circular motion settings below
#----------------------------------------------

# Used for specifying circular motion rotation angle
# Can be positive or negative
float32 rotation_angle

# Keep endeffector orientation if true (do not rotate around rotation_center_pose)
bool keep_endeffector_orientation
