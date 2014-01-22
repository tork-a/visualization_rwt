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
