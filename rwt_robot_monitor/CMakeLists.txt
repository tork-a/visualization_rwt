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

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/test_rwt_robot_monitor.test)
endif()
