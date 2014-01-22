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
  return (self.plotting_field !== null &&
          self.plotting_directories.length !== 0);
};

ROSLIB.DiagnosticsPlotInfo.prototype.plotFieldID = function() {
  var self = this;
  // generate random id if not set
  while (!self.plot_field_id) {
    var text = '';
    var possible = 'abcdefghijklmnopqrstuvwxyz';
    for( var i=0; i < 10; i++ ) {
      text += possible.charAt(Math.floor(Math.random() * possible.length));
    }
    if ($('#' + text).length === 0) {
      self.plot_field_id = text;
      break;
    }
  }
  return self.plot_field_id;
};

ROSLIB.DiagnosticsPlotInfo.prototype.preparePlotWindows = function(name, plot_windows_id) {
  var self = this;
  self.name = name;
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
  // adding root html
  self.$root_html = $('<div class="rwt-plot-info-container" id="' + self.plotFieldID() + '">'
                      + '<div class="plot-info container">'
                      +   '<form>'
                      +     '<div class="row">'
                      +       '<div class="col-xs-6">'
                      +          '<select class="form-control" disabled>'
                      +             '<option>' + name + '</option>'
                      +          '</select>'
                      +       '</div>'
                      +       '<div class=" col-xs-5">'
                      +          '<select class="form-control" disabled>'
                      +             '<option>'
                      +                self.plotting_field
                      +             '</option>'
                      +          '</select>'
                      +       '</div>'
                      +       '<div class=" col-xs-1">'
                      +       '<button class="btn btn-danger remove-info-button">remove</button>'
                      +       '</div>'
                      +     '</div>'
                      +   '</form>'
                      + '</div>'
                      + '<div class="windows-inner"></div>'
                      + '</div>');
  $('#' + plot_windows_id).prepend(self.$root_html);
  self.$root_html.find('.remove-info-button').click(function(e) {
    e.preventDefault();
    self.remove();
    return false;
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

  var $plot_area = self.$root_html.find('.windows-inner');
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

ROSLIB.DiagnosticsPlotInfo.prototype.remove = function() {
  var self = this;
  _.forEach(self.plot_windows_by_name, function(win) {
    win.remove();
  });
  $('#' + self.plotFieldID()).remove();
};
