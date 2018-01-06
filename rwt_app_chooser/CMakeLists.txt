cmake_minimum_required(VERSION 2.8.3)
project(rwt_app_chooser)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

## TODO: test
