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
  return rec(this).replace(' ', '-').replace('(', '').replace(')', '');
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
  $('#all-list li').remove();
  // return jquery object
  var rec = function(directory) {
    var $html = $('<li class="list-group-item inner">'
                  + '<a data-toggle="collapse" data-parent="#all-list" href="#' + directory.uniqID() + '">'
                  + directory.getCollapseIconHTML()
                  + directory.getIconHTML() + directory.name + '</a>'
                  +'</li>');
    if (directory.children.length === 0) {
      return $html;
    }
    else {
      var div_root = $('<ul class="list-group-item-content collapse" id="' + directory.uniqID() + '"></ul>');
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
