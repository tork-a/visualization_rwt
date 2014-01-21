// PlotWindow.js

/**
 * @fileOverview a file to define DiagnosticsPlotWindow class.
 * @author Ryohei Ueda
 */

ROSLIB.DiagnosticsPlotWindow = function(spec) {
  var self = this;
  self.directory = spec.directory;
};

ROSLIB.DiagnosticsPlotWindow.prototype.initialize = function(spec) {
  var self = this;
  self.index = spec.index;

  // creating html
  self.$html = $('<div class="rwt-diagnostics-plot-window col-xs-4"></div>');
  self.$html.data('index', self.index);
  
};

ROSLIB.DiagnosticsPlotWindow.prototype.getHTMLObject = function() {
  var self = this;
  return self.$html;
};

ROSLIB.DiagnosticsPlotWindow.prototype.update = function() {
};

ROSLIB.DiagnosticsPlotWindow.prototype.remove = function() {
};

