# The current robot state
moveit_msgs/RobotState current_state

# The robot goal state
moveit_msgs/MotionPlanRequest motion_plan_request

# The target duration of the motion
float64 duration

# The sample rate of the resulting trajectory
float64 trajectory_sample_rate 

# Specify whether the planner should check for self-collisions
bool check_self_collisions
