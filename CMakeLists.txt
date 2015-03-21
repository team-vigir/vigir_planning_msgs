cmake_minimum_required(VERSION 2.8.3)
project(vigir_planning_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  sensor_msgs
  std_msgs
)

set(MSG_DEPS
  std_msgs
  sensor_msgs
  moveit_msgs
  actionlib_msgs
)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})

set(MSG_FILES
ExtendedPlanningOptions.msg
JointPositionConstraint.msg
PlannerConfiguration.msg
)

set(SRV_FILES
#GetMotionPlan.srv
)

set(ACT_FILES
#MoveGroup.action
Move.action
)

add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
#add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})
