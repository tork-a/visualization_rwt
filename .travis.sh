#!/bin/bash

set -e

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

travis_time_start setup_apt_sources

if [[ "$ROS_DISTRO" =~ "indigo"|"jade" ]]; then
    sudo apt-get install -q -y software-properties-common curl python
    sudo add-apt-repository -y ppa:longsleep/python2.7-backports
    sudo apt-get update
    sudo apt-get dist-upgrade -y
    curl https://bootstrap.pypa.io/pip/2.7/get-pip.py | sudo -E python
fi

apt-get update -qq && apt-get install -y -q wget sudo lsb-release gnupg # for docker
DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata # https://stackoverflow.com/questions/44331836/apt-get-install-tzdata-noninteractive

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq || echo Ignore error on apt-get update

travis_time_end
travis_time_start install_ros

sudo apt-get install -qq -y python-catkin-pkg python-catkin-tools python-pip python-rosdep python-wstool
sudo apt-get install -qq -y ros-$ROS_DISTRO-ros ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-common-tutorials ros-$ROS_DISTRO-rospy-tutorials ros-$ROS_DISTRO-actionlib-tutorials
sudo apt-get install -qq -y ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-tf2-web-republisher

sudo apt-get install -qq -y firefox-geckodriver

source /opt/ros/$ROS_DISTRO/setup.bash

travis_time_end
# https://docs.travis-ci.com/user/gui-and-headless-browsers/
# travis_time_start start_xfvb

# # Set up Xfvb for Firefox headless testing
# export DISPLAY=:99.0
# sh -e /etc/init.d/xvfb start

# travis_time_end
travis_time_start setup_catkin_workspace

# Setup Catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws && catkin init
wstool init src
wstool merge -t src ${CI_SOURCE_PATH}/.rosinstall
wstool update -t src
ln -s $CI_SOURCE_PATH src/${REPOSITORY_NAME}
ls -al src/
wstool info -t src

travis_time_end
travis_time_start rosdep_init_update

sudo rosdep init || sudo rosdep init
rosdep update --include-eol-distros 
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y -n

travis_time_end
travis_time_start catkin_build

catkin build
source devel/setup.bash

travis_time_end

travis_time_start catkin_test

catkin run_tests -p1
catkin_test_results --verbose --all build

travis_time_end
