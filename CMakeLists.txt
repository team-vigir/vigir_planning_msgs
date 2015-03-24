cmake_minimum_required(VERSION 2.8.3)
project(vigir_planning_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  moveit_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
)

set(MSG_DEPS
  std_msgs
  sensor_msgs
  moveit_msgs
  geometry_msgs
  actionlib_msgs
)

#find_package(catkin REQUIRED genmsg ${MSG_DEPS})

set(MSG_FILES
  ExtendedPlanningOptions.msg
  RequestDrakeIK.msg
  ResultDrakeIK.msg
  RequestDrakeTrajectory.msg
  ResultDrakeTrajectory.msg
  RequestDrakeCartesianTrajectory.msg  
  ExtendedPlanningOptions.msg
  JointPositionConstraint.msg
  PlannerConfiguration.msg
)

set(SRV_FILES
  #GetMotionPlan.srv
  RequestWholeBodyIK.srv
  RequestWholeBodyTrajectory.srv
  RequestWholeBodyCartesianTrajectory.srv
)

set(ACT_FILES
  #MoveGroup.action
  Move.action
)

add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS moveit_msgs sensor_msgs message_runtime std_msgs
  DEPENDS ${MSG_DEPS})
  
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
