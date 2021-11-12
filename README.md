visualization_rwt [![Build Status](https://app.travis-ci.com/tork-a/visualization_rwt.svg?branch=kinetic-devel)](https://app.travis-ci.com/tork-a/visualization_rwt)
=================

visualization packages based on RobotWebTools

## INSTALL

### Package Install (Recommended)

```
apt-get install ros-$ROS_DISTRO-visualization-rwt
```

### Source Install
Following is an example with ROS Indigo.

1. Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and navigate to its source directory (e.g. `~/catkin_ws/src`).

2. In your Catkin workspace, download source and build with the following commands.

```
cd ~/catkin_ws
wstool init src
cd src/
wstool set visualization_rwt --git https://github.com/tork-a/visualization_rwt/
wstool update
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y
cd ~/catkin_ws 
catkin_make                (or any build commands available in ROS, e.g. `catkin build`)
source devel/setup.bash
```

PROJECTS
--------

* [rwt_app_chooser](rwt_app_chooser/README.md)
* [rwt_image_view](rwt_image_view/README.md)
* [rwt_moveit](rwt_moveit/README.rst)
* [rwt_plot](rwt_plot/README.md)
* [rwt_speech_recognition](rwt_speech_recognition/README.md)
* [rwt_steer](rwt_steer/README.md)
* [rwt_nav](rwt_nav/README.md)
