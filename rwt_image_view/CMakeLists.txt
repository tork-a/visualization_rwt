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