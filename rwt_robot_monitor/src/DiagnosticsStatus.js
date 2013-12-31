// DiagnosticsStatus

/**
 * @fileOverview a file to define RWTRobotMonitor.DiagnosticsStatus class.
 * @author Ryohei Ueda
 */


/**
 * @class DiagnosticsStatus
 * @param spec
 */
ROSLIB.DiagnosticsStatus = function(spec) {
  
};


ROSLIB.DiagnosticsStatus.createFromArray = function(msg) {
  var header = msg.header;
  var header_stamp = ROSLIB.Time.fromROSMsg(header);
};
