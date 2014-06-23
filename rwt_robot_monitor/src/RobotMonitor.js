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
