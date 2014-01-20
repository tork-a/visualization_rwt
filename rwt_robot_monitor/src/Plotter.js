// Plotter.js

/**
 * @fileOverview a file to define RWTDiagnosticsPlotter class.
 * @author Ryohei Ueda
 */
ROSLIB.RWTDiagnosticsPlotter = function(spec) {
  var self = this;
  self.previous_directory_names = [];
  var ros = spec.ros;
  self.history = new ROSLIB.DiagnosticsHistory(spec);
  self.name_select_id = spec.name_select_id || 'name-select';
  self.plot_field_select_id = spec.plot_field_select_id || 'plot-field-select';
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
  var self =this;

};


ROSLIB.RWTDiagnosticsPlotter.prototype.registerPlotFieldSelectCallback = function() {
  var self = this;
  $('#' + self.plot_field_select_id).bind('change', function() {
    console.log($(this).val());
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
    }
    else {
      
      for (var key in directory.status.values) {
        var $option = $('<option>' + key + '</option>');
        $option.val(key);
        var value = directory.status.values[key];
        var number_value = Number(value);
        
        if (!isNaN(number_value)) {
          $('#' + self.plot_field_select_id).append($option);
        }
      }
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
  var directories = self.history.root.getAllDirectories();
  // remove the root
  _.remove(directories, function(dir) {
    return dir.parent === null;
  });
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
  }
  
};
