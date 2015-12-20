#!/bin/bash
## Originally maded at https://github.com/RobotWebTools/roslibjs/blob/develop/.travis.yml

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TORK
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#       * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#       * Neither the name of the TORK, nor the names
#       of its contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# ROS deps for examples
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-catkin-pkg python-catkin-tools python-rosdep python-wstool 
sudo apt-get install -qq -y ros-$ROS_DISTRO-ros ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-common-tutorials ros-$ROS_DISTRO-rospy-tutorials ros-$ROS_DISTRO-actionlib-tutorials
sudo apt-get install -qq -y ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-tf2-web-republisher 
source /opt/ros/$ROS_DISTRO/setup.bash

# Only update npm in the 0.8 and 0.10 versions to not run into this issue:
#   https://github.com/nodejs/node/issues/433
case ${TRAVIS_NODE_VERSION} in 0.8*|0.10*) npm update -g npm ;; esac

# Set up Xfvb for Firefox headless testing
export DISPLAY=:99.0
sh -e /etc/init.d/xvfb start

# Setup Catkin workspace
cd ${TRAVIS_BUILD_DIR} && mkdir src && cd src && catkin_init_workspace && cd ${TRAVIS_BUILD_DIR}
wstool init src
wstool merge -t src https://raw.githubusercontent.com/tork-a/visualization_rwt/hydro-devel/.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro indigo -r -y
catkin build
source devel/setup.bash
