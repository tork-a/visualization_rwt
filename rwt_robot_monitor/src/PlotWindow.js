// PlotWindow.js

/**
 * @fileOverview a file to define DiagnosticsPlotWindow class.
 * @author Ryohei Ueda
 */

ROSLIB.DiagnosticsPlotWindow = function(spec) {
  var self = this;
  self.directory = spec.directory;
};

ROSLIB.DiagnosticsPlotWindow.prototype.getDirectory = function() {
  var self = this;
  return self.directory;
};

ROSLIB.DiagnosticsPlotWindow.prototype.initializePlotter = function() {
  var self = this;
  self.plotter.initializePlot(self.$html.find('.plot-window-inner'), {
    margin: {
      left: 20,
      top: 2,
      bottom: 20,
      right: 2
    },
    yaxis: {
      auto_scale: true,
      auto_scale_margin: 0.2,
      min: 0.1,
      tick: 3
    }
  });
  self.plotter.clearData();
};


ROSLIB.DiagnosticsPlotWindow.prototype.initialize = function(spec) {
  var self = this;
  self.index = spec.index;
  self.plotter = new ROSLIB.RWTPlot({
    max_data: 10,
    timestamp: true
  });
  
  // creating html
  self.$html = $('<div class="rwt-diagnostics-plot-window col-xs-2"></div>');
  self.$html.data('index', self.index);
  self.$html.append('<div class="background"><p>' + self.directory.status.name +'</p></div>');
  self.$html.append('<div class="plot-window-inner" id="rwt-plot-window-' + self.index + '"></div>');
  self.$html.append('<button class="close-button-layer close" type="button">&times;</button>');
  self.$html.find('.close').click(function() {
    self.remove();
  });
};

ROSLIB.DiagnosticsPlotWindow.prototype.getHTMLObject = function() {
  var self = this;
  return self.$html;
};

ROSLIB.DiagnosticsPlotWindow.prototype.update = function(data) {
  var self = this;
  var now = ROSLIB.Time.now();
  self.plotter.addData(now, [Number(data)]);
  if (self.directory.status.isOK()) {
    self.plotter.setColor(d3.rgb('#5cb85c'));
  }
  else if (self.directory.status.isWARN()) {
    self.plotter.setColor(d3.rgb('#f0ad4e'));
  }
  else if (self.directory.status.isERROR()) {
    self.plotter.setColor(d3.rgb('#d9534f'));
  }
};

ROSLIB.DiagnosticsPlotWindow.prototype.remove = function() {
  var self = this;
  self.$html.remove();
  self.$html = null;
};

