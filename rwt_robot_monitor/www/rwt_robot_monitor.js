// DiagnosticsDirectory.js

/**
 * @fileOverview a file to define RWTRobotMonitor.DiagnosticsDirectory class
 * @author Ryohei Ueda
 */

/**
 * @class DiagnosticsDirectory
 * @param spec
 */
ROSLIB.DiagnosticsDirectory = function(spec, parent, children) {
  if (typeof parent === 'undefined') {
    parent = null;
  }
  if (typeof children === 'undefined') {
    children = [];
  }

  this.parent = parent;
  this.children = children;
  this.status = spec.status;
  this.name = spec.name;
};

/**
 * lookup the directory whose name equals to `name'. This method digs the children
 * of the directory. If succeeeds to find that, returns the instance
 * of ROSLIB.DiagnosticsDirectory. if failed, returns null.
 * @param name - the name of directory
 */
ROSLIB.DiagnosticsDirectory.prototype.findDirectory = function(name) {
  if (this.name.toString() === name.toString()) {
    return this;
  }
  else if (this.children.length === 0) { // no children
    return null;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].findDirectory(name);
      if (child_result !== null) {
        return child_result;
      }
    }
    return null;
  }
};

/**
 * add child directory to this directory
 * @param directory - an instance of ROSLIB.DiagnosticsDirectory
 */
ROSLIB.DiagnosticsDirectory.prototype.addChild = function(directory) {
  this.children.push(directory);
  directory.parent = this;
};

/**
 * create a child directory which has this directory as parent
 * @ param name - name of child directory
 */
ROSLIB.DiagnosticsDirectory.prototype.createChild = function(name) {
  var child = new ROSLIB.DiagnosticsDirectory({
    name: name
  }, this);
  this.addChild(child);
  return child;
};

/**
 * register a status to the directory
 * @param status - instance of ROSLIB.DiagnosticsStatus
 */
ROSLIB.DiagnosticsDirectory.prototype.registerStatus = function(status) {
  this.status = status;
  return status;
};

/**
 * return the instance of directory if the directory has error instance 
 * as children.
 */
ROSLIB.DiagnosticsDirectory.prototype.isChildrenHasError = function() {
  if (this.isErrorStatus()) {
    return this;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].isChildrenHasError();
      if (child_result) {
        return child_result;
      }
    }
  }
};

/**
 * return true if the status registered to the directory has error level.
 */
ROSLIB.DiagnosticsDirectory.prototype.isErrorStatus = function() {
  if (this.status) {
    return this.status.isERROR();
  }
  else {
    return false;
  }
};

/**
 * return full path of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.fullName = function() {
  var rec = function(target_dir) {
    if (target_dir.parent === null) { // root
      return '';
    }
    else {
      var parent_result = rec(target_dir.parent);
      return parent_result + '/' + target_dir.name;
    }
  };
  return rec(this);
};

/**
 * get the uniq id which is available as html id
 */
ROSLIB.DiagnosticsDirectory.prototype.uniqID = function() {
  var rec = function(target_dir) {
    if (target_dir.parent === null) { // root
      return '';
    }
    else {
      var parent_result = rec(target_dir.parent);
      if (parent_result.length === 0) {
        return target_dir.name;
      }
      else {
        return parent_result + '-' + target_dir.name;
      }
    }
  };
  return rec(this).replace(' ', '-').replace('(', '').replace(')', '').replace('\t', '-');
};


/**
 * get an array of all the directories without root itself
 */
ROSLIB.DiagnosticsDirectory.prototype.getAllDirectoriesWithoutRoot = function() {
  var self = this;
  var directories = self.getAllDirectories();
  _.remove(directories, function(dir) {
    return dir === self;
  });
  return directories;
};

/**
 * get an array of all the directories
 */
ROSLIB.DiagnosticsDirectory.prototype.getAllDirectories = function() {
  var rec = function(target_dir) {
    if (target_dir.children.length === 0) {
      return [target_dir];
    }
    else {
      var result = [];
      for (var i = 0; i < target_dir.children.length; i++) {
        var child_result = rec(target_dir.children[i]);
        result = result.concat(child_result);
      }
      result.push(target_dir);
      return result;
    }
  };
  return rec(this);
};


/**
 * get an array of directories which has `level' such as error, warning and ok.
 */
ROSLIB.DiagnosticsDirectory.prototype.getDirectories = function(level) {
  var rec = function(target_dir) {
    if (target_dir.children.length === 0) {
      if (target_dir.status && target_dir.status.level === level) {
        return [target_dir];
      }
      else {
        return [];
      }
    }
    else {
      var result = [];
      for (var i = 0; i < target_dir.children.length; i++) {
        var child_result = rec(target_dir.children[i]);
        result = result.concat(child_result);
      }
      if (target_dir.status && target_dir.status.level === level) {
        result.push(target_dir);
      }
      return result;
    }
  };
  return rec(this);
};

/**
 * return an array of directories which has error status
 */
ROSLIB.DiagnosticsDirectory.prototype.getErrorDirectories = function() {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.ERROR);
};

/**
 * return an array of directories which has warn status
 */
ROSLIB.DiagnosticsDirectory.prototype.getWarnDirectories = function() {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.WARN);
};

/**
 * return an array of directories which has ok status
 */
ROSLIB.DiagnosticsDirectory.prototype.getOkDirectories = function() {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.OK);
};

/**
 * look up the directory by name throught directory tree return the instance of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.findByName = function(name) {
  if (this.status && this.status.name.toString() === name.toString()) {
    return this;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].findByName(name);
      if (child_result) {
        return child_result;
      }
    }
    return null;
  }
};

/**
 * return html to show icon suitable for error status of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.getIconHTML = function() {
  if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.OK) {
    return '<span class="glyphicon-ok glyphicon"></span>';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.WARN) {
    return '<span class="glyphicon-exclamation-sign glyphicon"></span>';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.ERROR) {
    return '<span class="glyphicon-minus-sign glyphicon"></span>';
  }
};

/**
 * return html of icon to show this directory has child
 */
ROSLIB.DiagnosticsDirectory.prototype.getCollapseIconHTML = function() {
  if (this.children.length !== 0) {
    return '<span class="glyphicon-chevron-right glyphicon"></span>';
  }
  else {
    return '';
  }
};


/**
 * return true if the directory has any children
 */
ROSLIB.DiagnosticsDirectory.prototype.hasChildren = function() {
  return this.children.length !== 0;
};

// DiagnosticsHistory

/**
 * @fileOverview a file to define RWTRobotMonitor.DiagnosticsHistory class.
 * @author Ryohei Ueda
 */

/**
 * @class DiagnosticsHistory
 * @param spec
 */

ROSLIB.DiagnosticsHistory = function(spec) {
  if (typeof spec === 'undefined') {
    spec = {};
  }
  this.root = new ROSLIB.DiagnosticsDirectory({name: 'root'});
};

/**
 * adding a status to the history.
 * @param status - instance of DiagnosticsStatus
 */
ROSLIB.DiagnosticsHistory.prototype.registerStatus = function(status) {
  // parse paths
  // lookup directory to insert into
  var parent_dir = this.root;
  for (var i = 0; i < status.path.length; i++) {
    var the_dir = parent_dir.findDirectory(status.path[i]);
    if (the_dir === null) {
      the_dir = parent_dir.createChild(status.path[i]);
    }
    parent_dir = the_dir;
  }
  // finally, parent_dir should point to the directory
  // the status should be inserted into
  parent_dir.registerStatus(status);
  return parent_dir;
};


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
  if (typeof spec === 'undefined') {
    spec = {};
  }
  this.name = spec.name;
  this.message = spec.message;
  this.level = spec.level;
  this.hardware_id = spec.hardware_id;
  this.values = {};
  this.stamp = spec.timestamp;
  for (var i = 0; i < spec.values.length; i++) {
    this.values[spec.values[i].key] = spec.values[i].value;
  }

  // parsing name
  // name has a directory separated by /
  this.path = _.filter(this.name.split('/'), function(str) {
    return str.toString() !== ''.toString();
  });
};

ROSLIB.DiagnosticsStatus.LEVEL = {
  OK: 0,
  WARN: 1,
  ERROR: 2
};

/**
 * return true if the level is OK
 */
ROSLIB.DiagnosticsStatus.prototype.isOK = function() {
  return this.level === ROSLIB.DiagnosticsStatus.LEVEL.OK;
};

/**
 * return true if the level is WARN
 */
ROSLIB.DiagnosticsStatus.prototype.isWARN = function() {
  return this.level === ROSLIB.DiagnosticsStatus.LEVEL.WARN;
};

/**
 * return true if the level is ERROR
 */
ROSLIB.DiagnosticsStatus.prototype.isERROR = function() {
  return this.level === ROSLIB.DiagnosticsStatus.LEVEL.ERROR;
};

/**
 * create DiagnosticsStatus instances from DiagnosticArray
 */
ROSLIB.DiagnosticsStatus.createFromArray = function(msg) {
  var header = msg.header;
  var header_stamp = ROSLIB.Time.fromROSMsg(header);
  return _.map(msg.status, function(status) {
    return new ROSLIB.DiagnosticsStatus({
      timestamp: header_stamp,
      name: status.name,
      message: status.message,
      hardware_id: status.hardware_id,
      level: status.level,
      values: status.values
    });
  });
};

/**
 * return the level as string
 */

ROSLIB.DiagnosticsStatus.prototype.levelString = function() {
  if (this.isERROR()) {
    return 'Error';
  }
  else if (this.isWARN()) {
    return 'Warn';
  }
  else if (this.isOK()){
    return 'OK';
  }
};

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


// Plotter.js

/**
 * @fileOverview a file to define RWTDiagnosticsPlotter class.
 * @author Ryohei Ueda
 */
ROSLIB.RWTDiagnosticsPlotter = function(spec) {
  var self = this;
  self.plotting_infos = [];
  self.previous_directory_names = [];
  var ros = spec.ros;
  self.history = new ROSLIB.DiagnosticsHistory(spec);
  self.name_select_id = spec.name_select_id || 'name-select';
  self.plot_field_select_id = spec.plot_field_select_id || 'plot-field-select';
  self.add_button_id = spec.add_button_id || 'add-button';
  self.plot_windows_id = spec.plot_windows_id || 'plot-windows-area';
  var diagnostics_agg_topic = spec.diagnostics_agg_topic || '/diagnostics_agg';
  
  self.registerNameSelectCallback();
  self.registerPlotFieldSelectCallback();
  self.registerAddCallback();
  self.diagnostics_agg_subscriber = new ROSLIB.Topic({
    ros: ros,
    name: diagnostics_agg_topic,
    messageType: 'diagnostic_msgs/DiagnosticArray'
  });
  
  self.diagnostics_agg_subscriber.subscribe(function(msg) {
    self.diagnosticsCallback(msg);
  });
};


ROSLIB.RWTDiagnosticsPlotter.prototype.registerAddCallback = function() {
  var self = this;
  $('#' + self.add_button_id).click(function(e) {
    var plotting_info = new ROSLIB.DiagnosticsPlotInfo();
    self.plotting_infos.push(plotting_info);
    plotting_info.clearInfo(); // clear it anyway
    var name = $('#' + self.name_select_id).val();
    var directory = self.history.root.findByName(name);
    var directories = [];
    if (directory.hasChildren()) {
      directories = directory.getAllDirectoriesWithoutRoot();
    }
    else {
      directories = [directory];
    }
    
    plotting_info.registerDirectories(directories);
    plotting_info.registerField($('#' + self.plot_field_select_id).val());
    plotting_info.preparePlotWindows(name, self.plot_windows_id);
    e.preventDefault();
    return false;
  });
};


ROSLIB.RWTDiagnosticsPlotter.prototype.registerPlotFieldSelectCallback = function() {
  var self = this;
  $('#' + self.plot_field_select_id).bind('change', function() {
  });
};

ROSLIB.RWTDiagnosticsPlotter.prototype.registerNameSelectCallback = function() {
  var self = this;
  $('#' + self.name_select_id).bind('change', function() {
    var name = $(this).attr('value');
    // get the directory
    var directory = self.history.root.findByName(name);
    var candidate_keys = [];
    $('#' + self.plot_field_select_id + ' option').remove();
    var invoke_error_message = false;
    if (directory.hasChildren()) {
      // get the children
      var children = directory.getAllDirectories();
      _.remove(children, function(dir) {
        return dir === directory;
      });
      var uniq_keys = _(children).map(function(dir) {
        var keys = [];
        for (var key in dir.status.values) {
          if (dir.status.values.hasOwnProperty(key)) {
            var value = dir.status.values[key];
            if (!isNaN(Number(value))) {
              keys.push(key);
            }
          }
        }
        return keys;
      }).flatten().uniq().value();
      uniq_keys.forEach(function(key) {
        var $option = $('<option>' + key + '</option>');
        $option.val(key);
        $('#' + self.plot_field_select_id).append($option);
      });
      if (uniq_keys.length === 0) {
        invoke_error_message = true;
      }
    }
    else {
      var counter = 0;
      for (var key in directory.status.values) {
        var $option = $('<option>' + key + '</option>');
        $option.val(key);
        var value = directory.status.values[key];
        var number_value = Number(value);
        
        if (!isNaN(number_value)) {
          $('#' + self.plot_field_select_id).append($option);
          counter = counter + 1;
        }
      }
      if (counter === 0) {
        invoke_error_message = true;
      }
    }
    if (invoke_error_message) {
      var $modal_html = $('<div class="modal fade" id="rwt-robot-plotter-warn-message">'
                          + '<div class="modal-dialog">'
                          + '<div class="modal-content">'
                          + '<div class="modal-header">'
                          + '<button type="button" class="close" data-dismiss="modal" aria-hidden="true">&times;</button>'
                          + '<h4 class="modal-title">Error</h4>'
                          + '</div>'
                          + '<div class="modal-body">'
                          + '<p><span class="label label-warning">'
                          + name
                          + '</span> does not have values which can be plotted</p>'
                          + '</div>'
                          + '<div class="modal-footer">'
                          + '<button type="button" class="btn btn-default" data-dismiss="modal">Close</button>'
                          + '</div>'
                          + '</div>'
                          + '</div>'
                          + '</div>');
      $('body').append($modal_html);
      // registering function to remove the html
      $modal_html.on('hidden.bs.modal', function() {
        $('#rwt-robot-plotter-warn-message').remove();
      });
      $('#rwt-robot-plotter-warn-message').modal();
    }
  });
  
};

ROSLIB.RWTDiagnosticsPlotter.prototype.diagnosticsCallback = function(msg) {
  var diagnostics_statuses
    = ROSLIB.DiagnosticsStatus.createFromArray(msg);
  var self = this;
  _.forEach(diagnostics_statuses, function(status) {
    self.history.registerStatus(status);
  });

  // sort the history
  var directories = self.history.root.getAllDirectoriesWithoutRoot();
  // sort self directories
  directories = _.sortBy(directories, function(dir) {
    return dir.fullName();
  });
  //var name_options = [];
  
  var need_to_update_options = false;
  if (self.previous_directory_names.length === 0) {
    need_to_update_options = true;
  }
  else if (self.previous_directory_names.length !== directories.length) {
    need_to_update_options = true;
  }
  else {
    for (var i = 0; i < self.previous_directory_names.length; i++) {
      if (self.previous_directory_names[i].toString() !==
          directories[i].fullName().toString()) {
        need_to_update_options = true;
        break;
      }
    }
  }
  
  if (need_to_update_options) {
    var has_field_before = true;
    if ($('#' + self.plot_field_select_id + ' option').length === 0) {
      has_field_before = false;
    }

    $('#' + self.name_select_id + ' option').remove();
    self.previous_directory_names = [];
    _.forEach(directories, function(dir) {
      var name = dir.fullName();
      //name_options.push('<option>' + dir.fullName() + '</option>');
      var $option = null;
      if (dir.hasChildren()) {
        var children = dir.getAllDirectories();
        $option = $('<option>' + name + '/* (' + (children.length - 1) +')</option>');
        
      }
      else {
        $option = $('<option>' + name + '</option>');
      }
      $option.attr('value', dir.fullName());
      $('#name-select').append($option);
      self.previous_directory_names.push(name);
    });
    
    if (!has_field_before) {
      // force to update field options
      $('#' + self.name_select_id).trigger('change');
    }
  }
  _.forEach(self.plotting_infos, function(plotting_info) {
    if (plotting_info.plottable()) {
      plotting_info.plot();
    }
  });
};

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

  this.history = new ROSLIB.DiagnosticsHistory(spec);
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
  var that = this;
  _.forEach(diagnostics_statuses, function(status) {
    that.history.registerStatus(status);
  });

  this.updateView();
  
};

/**
 * callback function to update string to show the last message received
 */
ROSLIB.RWTRobotMonitor.prototype.updateLastTimeString = function() {
  var that = this;
  if (that.last_diagnostics_update) {
    var now = ROSLIB.Time.now();
    var diff = now.substract(that.last_diagnostics_update).toSec();
    $(that.last_time_id).html(Math.floor(diff));
  }
  else {
    $(that.last_time_id).html(-1);
  }
  setTimeout(function() {
    that.updateLastTimeString();
  }, 1000);
};

/**
 * update html view
 */
ROSLIB.RWTRobotMonitor.prototype.updateView = function() {
  this.updateErrorList();
  this.updateWarnList();
  this.updateAllList();
  this.registerBrowserCallback();
};

ROSLIB.RWTRobotMonitor.prototype.updateList = function(list_id, level, icon) {
  $('#' + list_id + ' li').remove();
  var directories = this.history.root.getDirectories(level);
  directories.sort(function(a, b) {
    var apath = a.fullName();
    var bpath = b.fullName();
    if (apath > bpath) {
      return 1;
    }
    else if (bpath > apath) {
      return -1;
    }
    else {
      return 0;
    }
  });

  _.forEach(directories, function(dir) {
    var html_pre = '<li class="list-group-item" data-name="'
      + dir.fullName() + '"><span class="glyphicon ' + icon + '"></span>';
    var html_suf = '</li>';
    $('#' + list_id).append(html_pre
                            + dir.fullName() + ':' + dir.status.message
                            + html_suf);
  });
};

/**
 * update all list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateAllList = function() {
  // check opened list first
  var open_ids = [];
  $('#all-list .in, #all-list .collapsing').each(function() {
    open_ids.push($(this).attr('id'));
  });
  $('#all-list li').remove();
  // return jquery object
  var rec = function(directory) {
    var $html = $('<li class="list-group-item inner" data-name="' + directory.fullName() + '">'
                  + '<a data-toggle="collapse" data-parent="#all-list" href="#' + directory.uniqID() + '">'
                  + directory.getCollapseIconHTML()
                  + directory.getIconHTML() + directory.name + '</a>'
                  +'</li>');
    if (directory.children.length === 0) {
      return $html;
    }
    else {
      var div_root = $('<ul class="list-group-item-content collapse no-transition" id="' + directory.uniqID() + '"></ul>');
      for (var j = 0; j < open_ids.length; j++) {
        if (open_ids[j].toString() === directory.uniqID().toString()) {
          //div_root.find('.collapse').addClass('in');
          div_root.addClass('in');
          break;
        }
      }
      //if (directory.uniqID().toString() === )
      for (var i = 0; i < directory.children.length; i++) {
        var the_child = directory.children[i];
        var the_result = rec(the_child);
        div_root.append(the_result);
      }
      $html.append(div_root);
      return $html;
    }
  };

  for (var i = 0; i < this.history.root.children.length; i++) {
    var $html = rec(this.history.root.children[i]);
    $('#all-list').append($html);
  }
};

/**
 * update warn list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateWarnList = function() {
  this.updateList('warn-list', ROSLIB.DiagnosticsStatus.LEVEL.WARN, 'glyphicon-exclamation-sign');
};

/**
 * update error list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateErrorList = function() {
  this.updateList('error-list', ROSLIB.DiagnosticsStatus.LEVEL.ERROR, 'glyphicon-minus-sign');
};

/**
 * registering callbacks for clicking the view lists
 */
ROSLIB.RWTRobotMonitor.prototype.registerBrowserCallback = function() {
  var root = this.history.root;
  $('.list-group-item').dblclick(function() {
    if ($(this).find('.in').length !== 0) {
      return;                   // skip
    }
    var html = '<div class="modal fade" id="modal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">'
      + '<div class="modal-dialog">'
      + '<div class="modal-content">'
      + '<div class="modal-header">'
      + '<button type="button" class="close dismiss-button" aria-hidden="true">&times;</button>'
      + '<h4 class="modal-title" id="myModalLabel">Modal title</h4>'
      + '</div>'
      + '<div class="modal-body">'
      + '</div>'
      + '<div class="modal-footer">'
      + '<button type="button" class="btn btn-primary dismiss-button">Close</button>'
      + '</div>'
      + '</div><!-- /.modal-content -->'
      + '</div><!-- /.modal-dialog -->'
      +' </div><!-- /.modal -->';
    var the_directory = root.findByName($(this).attr('data-name'));
    var $html = $(html);
    var $first_body_html = $('<dl></dl>');
    var first_dict = {
      'Full name': the_directory.fullName(),
      'Component': the_directory.status.name,
      'Hardware ID': the_directory.status.hardware_id,
      'Level': the_directory.status.levelString(),
      'Message': the_directory.status.message
    };
    for (var first_key in first_dict) {
      $first_body_html.append('<dt>' + first_key + ':</dt>' + '<dd>' + first_dict[first_key] + '</dd>');
    }
    $html.find('.modal-body').append($first_body_html);
    var $second_body_html = $('<dl></dl>');
    for (var second_key in the_directory.status.values) {
      $second_body_html.append('<dt>' + second_key + ':</dt>' + '<dd>' + the_directory.status.values[second_key] + '</dd>');
    }
    $html.find('.modal-title').html(the_directory.fullName());
    
    $html.find('.modal-body').append($second_body_html);
    $html.find('.dismiss-button').click(function() {
      $html.on('hidden.bs.modal', function() {
        $('#modal').remove();
      });
      $html.modal('hide');
    });
    //$html.find('.modal-title').html()
    $('.container').append($html);
    $('#modal').modal();
    
  });
};
