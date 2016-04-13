cmake_minimum_required(VERSION 2.8.3)
project(rwt_plot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbridge_server
  rospy
  rwt_utils_3rdparty
  std_msgs
  roswww
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rosbridge_server rospy std_msgs rwt_utils_3rdparty roswww
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/array.py
  scripts/cos.py
  scripts/random_float.py
  scripts/random_point.py
  scripts/sin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)
