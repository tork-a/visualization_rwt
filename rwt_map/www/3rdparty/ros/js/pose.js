var NAV2D = NAV2D || {
  REVISION : '0.3.0'
};
NAV2D.Navigator = function(options) {
    var that = this;
    options = options || {};
    var ros = options.ros;
    var withOrientation = options.withOrientation || false;
    this.rootObject = options.rootObject || new createjs.Container();
  
    // get a handle to the stage
    var stage;
    if (that.rootObject instanceof createjs.Stage) {
      stage = that.rootObject;
    } else {
      stage = that.rootObject.getStage();
    }
  
    // marker for the robot
    var robotMarker = new ROS2D.NavigationArrow({
      size : 0.5,
      strokeSize : 0.01,
      fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),
      pulse : true
    });
    // wait for a pose to come in first
    robotMarker.visible = false;
    this.rootObject.addChild(robotMarker);
    var initScaleSet = false;
  
    // setup a listener for the robot pose
    var poseListener = new ROSLIB.Topic({
      ros : ros,
      name : '/robot_pose',
      messageType : 'geometry_msgs/Pose',
      throttle_rate : 100
    });
    poseListener.subscribe(function(pose) {
      // update the robots position on the map
      robotMarker.x = pose.position.x;
      robotMarker.y = -pose.position.y;
      if (!initScaleSet) {
        robotMarker.scaleX = 1.0 / stage.scaleX;
        robotMarker.scaleY = 1.0 / stage.scaleY;
        initScaleSet = true;
      }
  
      // change the angle
      robotMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
  
      robotMarker.visible = true;
    });
}