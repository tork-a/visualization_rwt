// RobotMonitor.js

/**
 * @fileOverview a file to define RWTRobotMonitor class.
 * @author Ryohei Ueda
 */

/**
 * a class to visualize diagnostics messages
 * @class RWTRobotMonitor
 * @param spec
 */
ROSLIB.RWTRobotMonitor = function(spec) {
  // spec, ros
  var diagnostics_agg_topic = spec.diagnostics_agg_topic || '/diagnostics_agg';
  var ros = spec.ros;
  this.last_diagnostics_update = null;
  this.last_time_id = spec.last_time_id;
  this.diagnostics_agg_subscriber = new ROSLIB.Topic({
    ros: ros,
    name: diagnostics_agg_topic,
    messageType: 'diagnostic_msgs/DiagnosticArray'
  });
  this.diagnostics_agg_subscriber.subscribe(this.diagnosticsCallback);

  // timer to update last_time_id
  var that = this;
  setTimeout(function() {
    that.updateLastTimeString();
  }, 1000);
};

/**
 * callback function for /diagnostics_agg.
 * @param msg - message of /diagnostics_agg.
 */
ROSLIB.RWTRobotMonitor.prototype.diagnosticsCallback = function(msg) {
  console.log('hogee');
  // var diagnostics_statuses
  //   = ROSLIB.RWTRobotMonitor.DiagnosticsStatus.createFromArray(msg);
};

/**
 * callback function to update string to show the last message received
 */
ROSLIB.RWTRobotMonitor.prototype.updateLastTimeString = function() {
  var that = this;
  if (this.last_diagnostics_update) {
    var now = ROSLIB.Time.now();
    var diff = now.substract(this.last_diagnostics_update).toSec();
    $(this.last_time_id).html(Math.floor(diff));
  }
  else {
    $(this.last_time_id).html(-1);
  }
  setTimeout(function() {
    that.updateLastTimeString();
  }, 1000);
};
