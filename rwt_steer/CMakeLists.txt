cmake_minimum_required(VERSION 2.8.3)
project(rwt_steer)

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

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/test_rwt_steer.test)
endif()
