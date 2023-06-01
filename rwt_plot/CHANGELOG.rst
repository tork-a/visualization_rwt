^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rwt_plot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

* [rwt_utils_3rdparty] update jquery from 1.8.3 to 1.12.4 (`#124 <https://github.com/tork-a/visualization_rwt//issues/124>`_)
* [rwt_plot][rwt_image_view] use rwt_utils_3rdparty/www/jquery.min.js instead of rwt_utils_3rdparty/www/jquery/jquery.min.js
* Contributors: Kei Okada, Koki Shinjo

0.1.1 (2021-11-12)
------------------

0.1.0 (2021-11-12)
------------------
* use websocket_port 9090 as defaut (`#112 <https://github.com/tork-a/visualization_rwt/issues/112>`_)
* add test for rwt tools (`#110 <https://github.com/tork-a/visualization_rwt/issues/110>`_)
* Contributors: Kei Okada, Shingo Kitagawa

0.0.5 (2021-03-12)
------------------

0.0.4 (2021-03-09)
------------------
* Add settings for ROS connection (`#76 <https://github.com/tork-a/visualization_rwt//issues/76>`_)
* Modified rwt's javascript module path. (`#70 <https://github.com/tork-a/visualization_rwt//issues/70>`_)
* Contributors: Furuta Yuki, Kei Okada, Iory Yanokura

0.0.3 (2016-10-01)
------------------

0.0.2 (2016-10-01)
------------------
* remove old rosbuild files
* reomve depends to the packages released from CDN
* Add maintainer. Consistent pkg versions. Sort out manifest format.
* use tork-a/roswww
* don't make functions within a loop
* catkinize visualization_rwt packages
* changes the color according to the level of the statuses `#15 <https://github.com/tork-a/visualization_rwt/issues/15>`_
* fixing conflict
* supporting ticks `#15 <https://github.com/tork-a/visualization_rwt/issues/15>`_: 

  * `#15 <https://github.com/tork-a/visualization_rwt/issues/15>`_: plotting data
  * `#17 <https://github.com/tork-a/visualization_rwt/issues/17>`_: using color category of d3
  * `#16 <https://github.com/tork-a/visualization_rwt/issues/16>`_: formatting second from epic time
  * `#16 <https://github.com/tork-a/visualization_rwt/issues/16>`_: implementing x-scale moving
  * `#16 <https://github.com/tork-a/visualization_rwt/issues/16>`_: fixing the increment bug
  * fixing xaxis scalling when the data is not enough to fill max_data
  * using timestamp for x axis
  * `#12 <https://github.com/tork-a/visualization_rwt/issues/12>`_: sorting the topics
  * fix default http port from 9090 -> 8888
  * add rosdep rosbridge_suite
  * `#8 <https://github.com/tork-a/visualization_rwt/issues/8>`_: using timestamp of header if possible
  * `#7 <https://github.com/tork-a/visualization_rwt/issues/7>`_: updating README
  * `#5 <https://github.com/tork-a/visualization_rwt/issues/5>`_: introducing new function: axisMinMaxWithMargin
  * `#5 <https://github.com/tork-a/visualization_rwt/issues/5>`_: adding auto_scale_margin
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: using floor
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: fixing rwt_plot
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: supporting timestamp data
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: supporting ros
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: supporting auto scale
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: deprecate multi plot2
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: supporting multi plot
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: supporting basic_sample
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: implementing rwt_plot using d3
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: adding less
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: adding d3
  * `#4 <https://github.com/tork-a/visualization_rwt/issues/4>`_: fixing indentation
  * `#3 <https://github.com/tork-a/visualization_rwt/issues/3>`_: fixing ROSLIB.Time in example
  * `#3 <https://github.com/tork-a/visualization_rwt/issues/3>`_: using RingBuffer for sequential data
  * `#3 <https://github.com/tork-a/visualization_rwt/issues/3>`_: implementing RingBuffer
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding some documentation
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding Profile.js
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: running jshint
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding grunt-contrib-lss
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding default task
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding jshint task
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding build task
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding Gruntfile.js which include concat and uglify task
  * `#2 <https://github.com/tork-a/visualization_rwt/issues/2>`_: adding package.json

* renaming js_utils -> rwt_utils_3rdparty
* the very first version of rwt_plot
* Contributors: Yuki Furuta, Isaac Saito, Kei Okada, Ryohei Uead
