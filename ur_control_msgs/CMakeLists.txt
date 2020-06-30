cmake_minimum_required(VERSION 3.1)
project(ur_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  # actionlib
  message_generation
  # std_msgs
)

# add_message_files(
#   DIRECTORY msg
#   FILES
#     x.msg
# )

add_service_files(
  DIRECTORY srv
  FILES
    SetServoJGain.srv
    SetServoJLookaheadTime.srv
    SetServoLoopRate.srv
    SetTeachModeEnabled.srv
)

# add_action_files(
#   DIRECTORY action
#   FILES
#     x.action
# )

generate_messages(
  # DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
