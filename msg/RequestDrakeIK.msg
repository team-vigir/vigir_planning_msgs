# The end-effectors' goal poses
geometry_msgs/PoseStamped[] target_poses

# Poses relative to end-effector that should reach the target poses
geometry_msgs/Point[] pose_on_eef

# The end-effectors' link names
string[] target_link_names

# The current robot state
moveit_msgs/RobotState robot_state

# Moveable joints
string[] free_joint_names