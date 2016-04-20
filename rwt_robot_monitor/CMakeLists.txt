cmake_minimum_required(VERSION 2.8.3)
project(rwt_robot_monitor)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  rosbridge_server
  roswww
  rwt_plot
  rwt_utils_3rdparty
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs rosbridge_server roswww rwt_plot rwt_utils_3rdparty
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)

install(DIRECTORY doc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www/doc
)
