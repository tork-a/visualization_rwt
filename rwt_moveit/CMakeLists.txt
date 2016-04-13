cmake_minimum_required(VERSION 2.8.3)
project(rwt_moveit)

find_package(catkin REQUIRED COMPONENTS
  interactive_marker_proxy
  kdl_parser
  moveit_msgs
  robot_state_publisher
  rosbridge_server
  rospy
  roswww
  rwt_utils_3rdparty
  sensor_msgs
  std_msgs
  tf
  tf2_web_republisher
  visualization_msgs
  message_generation
)

# Workaround for this issue http://answers.ros.org/question/107346/could-not-find-a-configuration-file-for-package/
find_package(orocos_kdl REQUIRED)

add_message_files(
  FILES
  MoveGroupPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS interactive_marker_proxy kdl_parser moveit_msgs robot_state_publisher rosbridge_server rospy roswww rwt_utils_3rdparty sensor_msgs std_msgs tf tf2_web_republisher visualization_msgs message_runtime
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
