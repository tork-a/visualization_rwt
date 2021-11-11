cmake_minimum_required(VERSION 2.8.3)
project(rwt_image_view)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roswww
  rwt_utils_3rdparty
  std_srvs
)

add_service_files(
  FILES
  RosbagRecordRequest.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS rospy rwt_utils_3rdparty std_srvs roswww message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/rosbag_record_server.py
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
  list(REMOVE_ITEM LAUNCH_FILES "${PROJECT_SOURCE_DIR}/launch/example.launch")
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/test_rwt_image_view.test)
endif()
