visualization_rwt [![Build Status](https://api.travis-ci.org/tork-a/visualization_rwt.png)](https://travis-ci.org/tork-a/visualization_rwt)
=================

visualization packages based on RobotWebTools

INSTALL
-------

Following is an example with ROS Indigo.

1. Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and navigate to its source directory (e.g. `~/catkin_ws/src`).

2. In your Catkin workspace, download source and build with the following commands.

```
cd ~/catkin_ws
apt-get install python-wstool
wstool init src
wstool merge -t src https://raw.githubusercontent.com/tork-a/visualization_rwt/hydro-devel/.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro indigo -r -y
catkin_make                (or any build commands available in ROS, e.g. `catkin build`)
source devel/setup.bash
```

PROJECTS
--------

* [rwt_plot](rwt_plot/README.md)
* [rwt_speech_recognition](rwt_speech_recognition/README.md)
