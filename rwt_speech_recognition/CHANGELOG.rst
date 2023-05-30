^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rwt_speech_recognition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2023-05-30)
------------------
* fix for noetic (`#130 <https://github.com/tork-a/visualization_rwt//issues/130>`_)

  * fix test code for selenium version >= 4.3.0

* Contributors: Kei Okada

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
* Contributors: Yuki Furuta, Kei Okada, Iory Yanokura

0.0.3 (2016-10-01)
------------------

0.0.2 (2016-10-01)
------------------
* Fix to work as of 2016/10/1 `#63 <https://github.com/tork-a/visualization_rwt/issues/63>`_

  * update README
  * update status texts
  * show selected language
  * do not overray mode when push continus/once
  * you can not use li value, use data-value http://stackoverflow.com/questions/31027506/get-value-attribute-from-an-li-element
  * fix rwt_speech_recognition.js path

* remove old rosbuild files
* reomve depends to the packages released from CDN
* Add maintainer. Consistent pkg versions. Sort out manifest format.
* use tork-a/roswww
* publish confidences; fix message type texts -> transcript `#48 <https://github.com/tork-a/visualization_rwt/issues/48>`_ 

  * remove .msg from message type
  * publish confidences; fix message type texts -> transcript
  * catkinize visualization_rwt packages
  * fix typo in index.html

* add rwt_speech_recognition
* Contributors: Yuki Furuta, Isaac Saito, Kei Okada
