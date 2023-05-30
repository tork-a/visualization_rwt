^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rwt_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2023-05-30)
------------------

* fix for noetic (`#130 <https://github.com/tork-a/visualization_rwt//issues/130>`_)

  * fix test code for selenium version >= 4.3.0

* move_base/goal requires frame_id, without starting '/' (`#119 <https://github.com/tork-a/visualization_rwt//issues/119>`_)

  * Warning: Invalid argument /map passed to canTransform argument source_frame in tf2 frame_ids cannot start with a '/' like:
    at line 134 in /tmp/binarydeb/ros-melodic-tf2-0.6.5/src/buffer_core.cpp
    [ WARN] [1645522151.660224428, 19134.759000000]: Failed to transform the goal pose from /map into the map frame: Invalid argument /map passed to lookupTransform argument source_frame in tf2 frame_ids cannot start with a '/' like:
    [ERROR] [1645522151.660344650, 19134.759000000]: The goal pose passed to this planner must be in the map frame.  It is instead in the /map frame.
    Closes `#118 <https://github.com/tork-a/visualization_rwt//issues/118>`_

* Contributors: Kei Okada

0.1.1 (2021-11-12)
------------------

0.1.0 (2021-11-12)
------------------
* use websocket_port 9090 as defaut (`#112 <https://github.com/tork-a/visualization_rwt/issues/112>`_)
* add test for rwt tools (`#110 <https://github.com/tork-a/visualization_rwt/issues/110>`_)
* Contributors: Kei Okada

0.0.5 (2021-03-12)
------------------

0.0.4 (2021-03-09)
------------------
* add rwt_nav package (`#96 <https://github.com/tork-a/visualization_rwt//issues/96>`_)
* Contributors: Kei Okada, Shingo Kitagawa, Yug Ajmera

0.0.3 (2016-10-01 15:52)
------------------------

0.0.2 (2016-10-01 15:20)
------------------------
