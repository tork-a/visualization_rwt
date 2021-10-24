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

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/test_rwt_plot.test)
endif()
