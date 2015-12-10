********************
Usage of rwt_moveit
********************

Prerequisite
===========================
 
 * (as of Nov 2013) Chrome Beta (for `WebGL` availability).  For Android https://play.google.com/store/apps/details?id=com.chrome.beta

Install, getting ready 
===========================

Install from source
------------------------

At the time of writing (Aug 2014) source install is the only way of getting the package work (better method is waited for).

1. Get the source.:

::

  $ cd %YOUR_CATKIN_WORKSPACE%/src
  $ git clone https://github.com/tork-a/rwt_ros.git
  $ git clone https://github.com/tork-a/visualization_rwt

2. Install dependencies of downloaded source.:

::

  $ rosdep install --from-paths src --ignore-src --rosdistro %YOUR_ROS_DISTRO% -y
  $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y     (example)

3. (This is just a temporary fix and should be removed in the very near future) Build web stuff:

::

  $ roscd rwt_moveit && make

 3-1. For `vs060` robot, as of Sep 22 2014, you might have to do the following BEFORE the commands above (solution discussed `here <https://github.com/tork-a/visualization_rwt/issues/43>`_):

 ::

   $ sudo mkdir `rospack find vs060`/www
   $ sudo ln -s -T `rospack find vs060`/model `rospack find vs060`/www/model

By now the package should be ready to serve for the robot packages.

At this point, the robot package you're using needs to be built. From here we take `vs060` as an example, which is already built and distributed as `DEB` package on Ubuntu so that you don't need to build on your own.

Setting
--------

1. Make sure `WebGL` is enabaled on your Chrome browser. See `chrome://flags/` and `WebGL: Hardware accelerated`.
2. On Chrome's flags page (type in URL bar `chrome://flags/`), enable the followings:

 * `Enable experimental canvas features`

Run, serve for a robot package
================================

Now let's run `rwt_moveit` with a robot package, for instance an industrial vertical arm vs060.

1. Install the robot package.

::

  $ apt-get install ros-%YOUR_ROS_DISTRO%-denso

2. On a terminal, run the robot with MoveIt! process.

::

  term-1$ roslaunch %ROBOT%_moveit_config demo_simulation.launch
  term-1$ roslaunch vs060_moveit_config demo_simulation_noenvironment.launch    (example with vs060)

3. On another terminal, run rwt_moveit process. Switch the `launch` file accordingly depending on whether you run the real robot or simulation.

::

  term-2$ roslaunch rwt_moveit demo.launch      (real robot)
  term-2$ roslaunch rwt_moveit sim_demo.launch  (simulation)

4. On the web browser, open:

::

  http://%HOSTNAME%:8000/rwt_moveit/
  http://localhost:8000/rwt_moveit/

Voila!

.. image:: http://wiki.ros.org/rwt_moveit?action=AttachFile&do=get&target=ros3djs_nextage.png
  :scale: 30%
  :alt: alternate text
  :align: left

Community
============

 * https://github.com/tork-a/rwt_ros
 * https://github.com/tork-a/visualization_rwt
