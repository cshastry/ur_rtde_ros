cmake_minimum_required(VERSION 2.8.3)
project(ur_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  #actionlib
  geometry_msgs
  message_generation
  std_msgs
  #trajectory_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    MoveJoint.msg
    MovePose.msg
    ServoJoint.msg
)

# add_service_files(
#   DIRECTORY srv
#   FILES
# )

# add_action_files(
#   DIRECTORY action
#   FILES
# )

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
