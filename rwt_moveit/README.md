Initial setting
========
Install necessary packages
tf2_web_republisher, interactive_marker_proxy, rosbridge_server

roscd rwt_moveit
make

How to work
========
Simlator
"""
roslaunch (ROBOT)_moveit_config demo_simulation.launch
roslaunch rwt_moveit sim_demo.launch
"""
active robot
"""
roslaunch (ROBOT)_moveit_config demo.launch
roslaunch rwt_moveit demo.launch
"""

Please access the following address:
http://(HOSTNAME):8000/rwt_moveit/
