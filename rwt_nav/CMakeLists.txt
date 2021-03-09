cmake_minimum_required(VERSION 2.8.3)
project(rwt_nav)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  roswww
  rwt_utils_3rdparty
)

catkin_package(
  CATKIN_DEPENDS rosbridge_server roswww rwt_utils_3rdparty
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
