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
  var that = this;
  this.diagnostics_agg_subscriber.subscribe(function(msg) {
    that.diagnosticsCallback(msg);
  });
  // timer to update last_time_id
  
  setTimeout(function() {
    that.updateLastTimeString();
  }, 1000);
};

/**
 * callback function for /diagnostics_agg.
 * @param msg - message of /diagnostics_agg.
 */
ROSLIB.RWTRobotMonitor.prototype.diagnosticsCallback = function(msg) {
  this.last_diagnostics_update = ROSLIB.Time.now();
  var diagnostics_statuses
    = ROSLIB.DiagnosticsStatus.createFromArray(msg);
  
};

/**
 * callback function to update string to show the last message received
 */
ROSLIB.RWTRobotMonitor.prototype.updateLastTimeString = function() {
  var that = this;
  console.log(this.last_diagnostics_update);
  if (that.last_diagnostics_update) {
    var now = ROSLIB.Time.now();
    var diff = now.substract(that.last_diagnostics_update).toSec();
    console.log(diff);
    $(that.last_time_id).html(Math.floor(diff));
  }
  else {
    $(that.last_time_id).html(-1);
  }
  setTimeout(function() {
    that.updateLastTimeString();
  }, 1000);
};
