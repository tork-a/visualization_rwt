# rwt_keyboard_teleop

### Control a ROS based Robot using on screen keyboard and speed control

### Task 1 : 

1. ~~Make the basic GUI for the web application and connect it with ROS.~~ (roslibjs and EventEmiiter2)
2. ~~Add keyboard/joystick functionality to the GUI.~~ (nipplejs used)
3. ~~Add Speed Control button and display current speed.~~
4. ~~Test it on a robot/simulation.~~ (Tested on [navros_pkg](https://github.com/YugAjmera/navros_pkg))
Run in different tabs:
```
roslaunch navros_pkg cafe_custom.launch
roslaunch rosbridge_server rosbridge_websocket.launch
```
- Open file "index.html"
- Control using Joystick

![](1.png)

### Task 2 :

1. ~~Add camera feed to the web page~~ : (Have used web_video_server for this.)
2. ~~Test it on a robot/simulation.~~ (Tested on [navros_pkg](https://github.com/YugAjmera/navros_pkg))
Run in different tabs:
```
roslaunch navros_pkg cafe_custom.launch
rosrun web_video_server web_video_server 
roslaunch rosbridge_server rosbridge_websocket.launch

```
- Open file "index.html"
- Control using Joystick
- Add camera topic
- Press Load video button

![](2.png)


