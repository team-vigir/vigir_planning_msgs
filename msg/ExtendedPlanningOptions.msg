uint8 TYPE_FREE_MOTION=1
uint8 TYPE_CARTESIAN_MOTION=2


# Specify the desired motion type
uint8[] target_motion_type

# Specify frame in which target poses below are specified
string target_frame

# Specify the desired waypoints. For default planning
# to target pose, just specify single target
geometry_msgs/Pose[] target_poses

# Specify if robot should continuously replan motion plan
bool continuous_replanning
