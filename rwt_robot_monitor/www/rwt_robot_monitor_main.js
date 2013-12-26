$(function() {
  var ros = new ROSLIB.Ros({
    url: "ws://" + location.hostname + ":8888"
  });
  var robot_monitor = new ROSLIB.RWTRobotMonitor({
    ros: ros,
    last_time_id: '#last-message-sec'
  });
});
