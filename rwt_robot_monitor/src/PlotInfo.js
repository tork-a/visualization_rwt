// PlotInfo.js

/**
 * @fileOverview a file to define DiagnosticsPlotInfo class.
 * @author Ryohei Ueda
 */

/**
 * DiagnosticsPlotInfo is a class to manager plotting information
 * including PlotWindow and so on.
 * You need to create DiagnosticsPlotInfo for each plotting fields.
 */
ROSLIB.DiagnosticsPlotInfo = function(spec) {
  var self = this;

  self.clearInfo();
};

ROSLIB.DiagnosticsPlotInfo.prototype.getDirectories = function() {
  var self = this;
  return _.filter(self.plotting_directories, function(dir) {
    if (dir.status.values.hasOwnProperty(self.plotting_field)) {
      return true;
    }
    else {
      return false;
    }
  });
};

ROSLIB.DiagnosticsPlotInfo.prototype.clearInfo = function() {
  var self = this;
  self.plotting_field = null;
  self.plotting_directories = [];
};

ROSLIB.DiagnosticsPlotInfo.prototype.registerDirectories = function(directories) {
  var self = this;
  self.plotting_directories = directories;
};

ROSLIB.DiagnosticsPlotInfo.prototype.registerField = function(field) {
  var self = this;
  self.plotting_field = field;
  return self.plotting_field;
};

ROSLIB.DiagnosticsPlotInfo.prototype.plotValues = function() {
  var self = this;
  var field = self.plotting_field;
  var values = {};
  _.forEach(self.plotting_directories, function(dir) {
    if (dir.status.values.hasOwnProperty(field)) {
      values[dir.fullName()] = dir.status.values[field];
    }
    else {
      values[dir.fullName()] = null;
    }
  });
  return {
    field: field,
    values: values
  };
};

ROSLIB.DiagnosticsPlotInfo.prototype.plottable = function() {
  var self = this;
  return (self.plotting_fields !== null &&
          self.plotting_directories.length !== 0);
};
