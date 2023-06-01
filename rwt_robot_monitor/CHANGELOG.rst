^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rwt_robot_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2023-06-01)
------------------
* add missing test_depends (`#131 <https://github.com/tork-a/visualization_rwt//issues/131>`_)
  fixes https://build.ros.org/job/Nbin_uF64__rwt_app_chooser__ubuntu_focal_amd64__binary/1/console
  ```
  13:43:55 -- Using Python nosetests: /usr/bin/nosetests3
  13:43:55 -- catkin 0.8.10
  13:43:55 -- BUILD_SHARED_LIBS is on
  13:43:55 -- Could NOT find roslaunch (missing: roslaunch_DIR)
  13:43:55 -- Could not find the required component 'roslaunch'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
  13:43:55 CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  13:43:55   Could not find a package configuration file provided by "roslaunch" with
  13:43:55   any of the following names:
  13:43:55
  13:43:55     roslaunchConfig.cmake
  13:43:55     roslaunch-config.cmake
  13:43:55
  13:43:55   Add the installation prefix of "roslaunch" to CMAKE_PREFIX_PATH or set
  13:43:55   "roslaunch_DIR" to a directory containing one of the above files.  If
  13:43:55   "roslaunch" provides a separate development package or SDK, be sure it has
  13:43:55   been installed.
  13:43:55 Call Stack (most recent call first):
  13:43:55   CMakeLists.txt:13 (find_package)
  13:43:55
  13:43:55
  13:43:55 -- Configuring incomplete, errors occurred!
  13:43:55 See also "/tmp/binarydeb/ros-noetic-rwt-app-chooser-0.1
  ```
* Contributors: Kei Okada

0.1.2 (2023-05-30)
------------------
* fix for noetic (`#130 <https://github.com/tork-a/visualization_rwt//issues/130>`_)

  * fix test code for selenium version >= 4.3.0

* Contributors: Kei Okada

0.1.1 (2021-11-12)
------------------
* [rwt_robot_monitor] remove roslibjs_experimental from package.xml (`#114 <https://github.com/tork-a/visualization_rwt/issues/114>`_)
* Contributors: Kei Okada

0.1.0 (2021-11-12)
------------------
* rwt_robot_monitor : fix css to have better plot view (`#113 <https://github.com/tork-a/visualization_rwt/issues/113>`_)
* use websocket_port 9090 as defaut (`#112 <https://github.com/tork-a/visualization_rwt/issues/112>`_)
* add test for rwt tools (`#110 <https://github.com/tork-a/visualization_rwt/issues/110>`_)
* Contributors: Kei Okada, Shingo Kitagawa, Iory Yanokura

0.0.5 (2021-03-12)
------------------

0.0.4 (2021-03-09)
------------------

0.0.3 (2016-10-01)
------------------

0.0.2 (2016-10-01)
------------------
* remove old rosbuild files
* reomve depends to the packages released from CDN
* Add maintainer. Consistent pkg versions. Sort out manifest format.
* use tork-a/roswww
* catkinize visualization_rwt packages
* adding new package rwt_robot_monitor
* Contributors: Furuta Yuki, Isaac Saito, Kei Okada, Ryohei Ueda
