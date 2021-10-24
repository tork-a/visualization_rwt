cmake_minimum_required(VERSION 2.8.3)
project(rwt_nav)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
  roswww
  rwt_utils_3rdparty
  geometry_msgs
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS rosbridge_server roswww rwt_utils_3rdparty
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## copy robot_pose_publihser.cpp from https://github.com/GT-RAIL/robot_pose_publisher/
## Declare a cpp executable
add_executable(robot_pose_publisher src/robot_pose_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_pose_publisher
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)

# Mark executables and/or libraries for installation
install(TARGETS robot_pose_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/test_rwt_nav.test)
endif()
