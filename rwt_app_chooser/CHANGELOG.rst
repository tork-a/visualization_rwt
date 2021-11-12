^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rwt_app_chooser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
