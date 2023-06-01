^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rwt_app_chooser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [rwt_app_chooser] support app args (`#117 <https://github.com/tork-a/visualization_rwt//issues/117>`_)

  * It is introduced in https://github.com/PR2/app_manager/pull/27 and released as 1.3.0 on Nov, 8, 2021

* fix for noetic (`#130 <https://github.com/tork-a/visualization_rwt//issues/130>`_)

  * fix test code for selenium version >= 4.3.0

* Fix typo in disconnect notification (`#120 <https://github.com/tork-a/visualization_rwt//issues/120>`_)

* Contributors: Guilherme Affonso, Kei Okada, Koki Shinjo

0.1.1 (2021-11-12)
------------------

0.1.0 (2021-11-12)
------------------
* use websocket_port 9090 as defaut (`#112 <https://github.com/tork-a/visualization_rwt/issues/112>`_)
* Fix test code  (`#112 <https://github.com/tork-a/visualization_rwt/issues/112>`_)

  * use xpath to get Hello World Task Icon
  * wait until system actually reads HelloWorld task
  * support gui argument

* add test for rwt tools (`#110 <https://github.com/tork-a/visualization_rwt/issues/110>`_)
* Pr/add spot support (`#108 <https://github.com/tork-a/visualization_rwt/issues/108>`_)
* Contributors: Kei Okada, Koki Shinjo

0.0.5 (2021-03-12)
------------------
* add sample sccript/launch, update README.md (`#100 <https://github.com/tork-a/visualization_rwt/issues/100>`_ )

  * add sample/launch/sample.launch
  * add README.md to refer sample.launch
  * say.py needs voice and volume, see https://github.com/ros-drivers/audio_common/pull/167
  * use default robot-uri setting to ocalhost
  * if /robot/name rosparam is not set, use 'robot' as default name, see https://github.com/PR2/app_manager/blob/10638eaea566f6b6708f70fe1b952078cea4b23e/scripts/app_manager#L65
  * rwt_app_chooser: make sample directory install

* Contributors: Kei Okada

0.0.4 (2021-03-09)
------------------
* Register app running user name in rwt_app_chooser (`#95 <https://github.com/tork-a/visualization_rwt//issues/95>`_)
* [rwt_app_chooser] add hrp2, baxter and fetch icons (`#87 <https://github.com/tork-a/visualization_rwt//issues/87>`_)
* rwt_app_chooser: fix typo in README.md (`#71 <https://github.com/tork-a/visualization_rwt//issues/71>`_)
* update readme / add sample app (`#67 <https://github.com/tork-a/visualization_rwt//issues/67>`_)
* Contributors: Yuki Furuta, Shingo Kitagawa

0.0.3 (2016-10-01 15:52)
------------------------

0.0.2 (2016-10-01 15:20)
------------------------
