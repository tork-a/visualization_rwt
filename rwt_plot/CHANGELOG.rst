^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rwt_plot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
