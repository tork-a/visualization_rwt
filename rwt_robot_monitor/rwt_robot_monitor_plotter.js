$(function() {
  var ros = new ROSLIB.Ros()
  ros.install_config_button("config-button");

  var robot_monitor = new ROSLIB.RWTDiagnosticsPlotter({
    ros: ros,
    name_select_id: 'name-select',
    plot_field_select_id: 'plot-field-select'
  });
});
