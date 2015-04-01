# The end-effectors' goal poses
geometry_msgs/PoseStamped[] target_poses

# The end-effectors' link names
string[] target_link_names

# The current robot state
moveit_msgs/RobotState robot_state

# Moveable joints
string[] free_joint_names