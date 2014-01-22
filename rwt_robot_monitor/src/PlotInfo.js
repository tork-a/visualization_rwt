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
  self.plot_windows_by_name = {};
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

ROSLIB.DiagnosticsPlotInfo.prototype.preparePlotWindows = function(plot_windows_id) {
  var self = this;
  _.forEach(self.plot_windows_by_name, function(win) {
    win.remove();
  });
  self.plot_windows_by_name = {};
  _.forEach(self.getDirectories(), function(dir) {
    var new_window = new ROSLIB.DiagnosticsPlotWindow({
      directory: dir
    });
    self.plot_windows_by_name[dir.fullName()] = new_window;
  });
  self.rearrangePlotWindows(plot_windows_id);
};

ROSLIB.DiagnosticsPlotInfo.prototype.rearrangePlotWindows = function(plot_windows_id) {
    var self = this;
  // first of all, find the removed window
  var removed_windows = _.remove(_.values(self.plot_windows_by_name), function(win) {
    return win.getHTMLObject() === null;
  });
  _.forEach(removed_windows, function(win) {
    delete self.plot_windows_by_name[win.getDirectory().fullName()];
  });

  var $plot_area = $('#' + plot_windows_id);
  $plot_area.html('');
  var $row = null;
  var plot_windows = _.values(self.plot_windows_by_name);
  for (var j = 0; j < plot_windows.length; j++) {
    if (j % 6 === 0) {
      if ($row) {
        $plot_area.append($row);
      }
      $row = $('<div class="row"></div>');
    }
    plot_windows[j].initialize({
      index: j
    });
    $row.append(plot_windows[j].getHTMLObject());
  }
  if (plot_windows.length % 6 !== 0 || plot_windows.length === 6) {
    $plot_area.append($row);
  }
  for (var i = 0; i < plot_windows.length; i++) {
    plot_windows[i].initializePlotter();
  }
  $plot_area.find('.close').click(function() {
    self.rearrangePlotWindows(plot_windows_id);
  });
};

ROSLIB.DiagnosticsPlotInfo.prototype.plot = function() {
  var self = this;
  var field_values = self.plotValues();
  for (var dir_name in field_values.values) {
    var val = field_values.values[dir_name];
    if (val && !isNaN(val)) {
      if (self.plot_windows_by_name.hasOwnProperty(dir_name)) {
        self.plot_windows_by_name[dir_name].update(val);
      }
    }
  }
};
