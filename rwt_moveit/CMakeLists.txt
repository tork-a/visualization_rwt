cmake_minimum_required(VERSION 2.8.3)
project(rwt_moveit)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MoveGroupPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  nodes/interactive_moveit.py
  nodes/link_group_publisher
  nodes/sample_moveit_publisher.py
  nodes/joint_state_publisher
  nodes/moveit_publisher.py
  nodes/virtual_joint_state_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)
