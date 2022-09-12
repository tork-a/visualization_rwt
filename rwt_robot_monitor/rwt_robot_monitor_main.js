$(function() {
  var ros = new ROSLIB.Ros()
  ros.install_config_button("config-button");

  var robot_monitor = new ROSLIB.RWTRobotMonitor({
    ros: ros,
    last_time_id: '#last-message-sec'
  });
});
