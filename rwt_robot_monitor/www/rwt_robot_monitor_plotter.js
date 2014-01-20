$(function() {
  var ros = new ROSLIB.Ros({
    url: "ws://" + location.hostname + ":8888"
  });
  var robot_monitor = new ROSLIB.RWTDiagnosticsPlotter({
    ros: ros,
    name_select_id: 'name-select'
  });
});
