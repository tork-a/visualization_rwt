rwt_app_chooser
===============

App chooser client for web browsing device


### Usage

1. (On Robot) Launch app manager

    ```bash
    roscore &
    rosparam set robot/type pr2     # set robot type as rosparam
    rosparam set robot/type pr1012  # set robot name as rosparam
    roslaunch rwt_app_chooser app_manager.launch
    ```

2. (On Server) Launch rwt_app_chooser server

    ```bash
    roslaunch rwt_app_chooser rwt_app_chooser.launch
    ```
    
    This can be run on the robot
    You can now access to `http://<server ip>:8080/`

### Author

    Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
