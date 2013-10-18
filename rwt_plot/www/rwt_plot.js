// Plot.js

// jquery, jquery.flot, lodash.js and roslib.js are required to be loaded before.

// class
ROSLIB.RWTPlot = function(spec) {
  this.max_data = spec.max_data || 100; // defaults to 100
  this.data = [];
  this.plot = null;
  this.use_timestamp = spec.timestamp;
};
ROSLIB.RWTPlot.prototype.clearData = function() {
  this.data = [];
};

ROSLIB.RWTPlot.prototype.initializePlot = function($content, spec, data) {
  // creating plot object integrating with jquery object
  // `spec' is the spec for jquery.flot
  var default_y_axis = {
    min: 0,
    //max: 1.0,
    show: true
  };
  var default_x_axis = {
    min: 0,
    max: this.max_data,
    show: true
  };
  var spec_x_axis = _.extend(default_x_axis, spec.xaxis || {});
  var spec_y_axis = _.extend(default_y_axis, spec.yaxis || {});
  if (spec_x_axis.auto_scale) {
    this.x_auto_scale = true;
    spec_x_axis.min = null;
    spec_x_axis.max = null;
  }
  if (spec_y_axis.auto_scale) {
    this.y_auto_scale = true;
    spec_y_axis.min = null;
    spec_y_axis.max = null;
  }
  var new_spec = _.extend(spec, {xaxis: spec_x_axis, yaxis: spec_y_axis});
  this.plot = $.plot($content, [[[0, 200]]], new_spec);
};

ROSLIB.RWTPlot.prototype.addRawData = function(data) {
  // cut this.data if the data is longer than this.max_data
  if (this.data.length > this.max_data) {
    this.data = this.data.slice(1);
  }

  // check the dimension
  var data_dimension = _.isArray(data) ? data.length : 1;
  if (data_dimension == 1)
    data = [data];          // force to encapsulate into array
  this.data.push(data);

  // auto scalling
  
  var plot_data = [];
  // plot_data := [[[x1, y1], [x1, y2], [x1, y3], ...], [[x1, z1], [x1, z2], ...], ...]
  for (var i = 0; i < this.data.length; i++) { // x_i := i
    for (var j = 0; j < this.data[i].length; j++) {
      var value = this.data[i][j];
      var new_data = [i, value]; // [x1, y1] or [x1, z1]
      if (plot_data.length <= j) {
        // adding new empty array to plot_data
        plot_data.push([]);
      }
      plot_data[j].push(new_data);
    }
  }
  if (this.plot) {
    this.plot.setData(plot_data);
  }
};

ROSLIB.ROSTimeToSec = function(timea) {
  return timea.secs + timea.nsecs / 1000000000.0;
};

ROSLIB.ROSTimeDifference = function(timea, timeb) {
  return (timea.secs - timeb.secs) + (timea.nsecs - timeb.nsecs) / 1000000000.0;
};

ROSLIB.RWTPlot.prototype.addTimestampedData = function(stamp, data) {
  // check the oldest message
  if (this.data.length > 0) {
    // chop here
    var chop_num = 0;
    for (var i = 0; i < this.data.length; i++) {
      var diff = ROSLIB.ROSTimeDifference(stamp, this.data[i].stamp);
      if (diff > this.max_data) {
        chop_num = chop_num + 1;
      }
      else {
        break;
      }
    }
    this.data = this.data.slice(chop_num);
  }
  
  var data_dimension = _.isArray(data) ? data.length : 1;
  if (data_dimension == 1)
    data = [data];          // force to encapsulate into array
  data.stamp = stamp;
  this.data.push(data);
  
  var plot_data = [];
  var oldest_stamp = this.data[0].stamp;
  // plot_data := [[[x1, y1], [x1, y2], [x1, y3], ...], [[x1, z1], [x1, z2], ...], ...]
  for (var i = 0; i < this.data.length; i++) { // x_i := i
    for (var j = 0; j < this.data[i].length; j++) {
      var value = this.data[i][j];
      // scale x(i) here
      // oldest_stamp = 0, stamp = this.max_data
      if (stamp != oldest_stamp) {
        var x = ROSLIB.ROSTimeDifference(this.data[i].stamp, oldest_stamp);
        var new_data = [x, value]; // [x1, y1] or [x1, z1]
        if (plot_data.length <= j) {
          // adding new empty array to plot_data
          plot_data.push([]);
        }
        plot_data[j].push(new_data);
      }
    }
  }
  if (this.plot) {
    this.plot.setData(plot_data);
  }
  
};

ROSLIB.RWTPlot.prototype.addData = function(data, data2) {
  if (this.use_timestamp) {
    this.addTimestampedData(data, data2);
  }
  else {
    this.addRawData(data);
  }
};

ROSLIB.RWTPlot.prototype.draw = function() {
  if (this.plot) {
    this.plot.setupGrid();
    this.plot.draw();
  }
};

// RoslibExt.js
/**
 * Retrieves a type of ROS topic.
 *
 * @param callback - function with params:
 *   * type - String of the topic type
 */
ROSLIB.Ros.prototype.getTopicType = function(topic, callback) {
  var topicTypeClient = new ROSLIB.Service({
    ros : this,
    name : '/rosapi/topic_type',
    serviceType : 'rosapi/TopicType'
  });
  var request = new ROSLIB.ServiceRequest({
    topic: topic
  });
  topicTypeClient.callService(request, function(result) {
    callback(result.type);
  });
};

/**
 * Retrieves a detail of ROS message.
 *
 * @param callback - function with params:
 *   * details - Array of the message detail
 * @param message - String of a topic type
 */
ROSLIB.Ros.prototype.getMessageDetails = function(message, callback) {
  var messageDetailClient = new ROSLIB.Service({
    ros : this,
    name : '/rosapi/message_details',
    serviceType : 'rosapi/MessageDetails'
  });
  var request = new ROSLIB.ServiceRequest({
    type: message
  });
  messageDetailClient.callService(request, function(result) {
    callback(result.typedefs);
  });
};

/**
 * Encode a typedefs into a dictionary like `rosmsg show foo/bar`
 * @param type_defs - array of type_def dictionary
 */
ROSLIB.Ros.prototype.decodeTypeDefs = function(type_defs) {
  var typeDefDict = {};
  var theType = type_defs[0];
  
  // It calls itself recursively to resolve type definition
  // using hint_defs.
  var decodeTypeDefsRec = function(theType, hint_defs) {
    var typeDefDict = {};
    for (var i = 0; i < theType.fieldnames.length; i++) {
      var arrayLen = theType.fieldarraylen[i];
      var fieldName = theType.fieldnames[i];
      var fieldType = theType.fieldtypes[i];
      if (fieldType.indexOf('/') === -1) { // check the fieldType includes '/' or not
        if (arrayLen === -1) {
          typeDefDict[fieldName] = fieldType;
        }
        else {
          typeDefDict[fieldName] = [fieldType];
        }
      }
      else {
        // lookup the name
        var sub_type = false;
        for (var j = 0; j < hint_defs.length; j++) {
          if (hint_defs[j].type.toString() === fieldType.toString()) {
            sub_type = hint_defs[j];
            break;
          }
        }
        if (sub_type) {
          var sub_type_result = decodeTypeDefsRec(sub_type, hint_defs);
          if (arrayLen === -1) {
            typeDefDict[fieldName] = sub_type_result;
          }
          else {
            typeDefDict[fieldName] = [sub_type_result];
          }
        }
        else {
          throw 'cannot find ' + fieldType;
        }
      }
    }
    return typeDefDict;
  };                            // end of decodeTypeDefsRec
  
  return decodeTypeDefsRec(type_defs[0], type_defs);
};

  ROSLIB.Time = function(spec) {
    this.nsecs = (spec || {}).nsecs || 0;
    this.secs = (spec || {}).secs || 0;
  };

  ROSLIB.Time.now = function() {
    var now = new Date();
    var msec = now.getTime();
    return new ROSLIB.Time({
      secs: Math.ceil(msec / 1000),
      nsecs: (msec % 1000) * 1000000
    });
  };

  ROSLIB.Time.prototype.toSec = function() {
      return this.secs + this.nsecs / 1000000000.0;
  };

  ROSLIB.Time.prototype.substract = function(another) {
    var sec_diff = this.secs - another.secs;
    var nsec_diff = this.nsecs - another.nsecs;
    if (nsec_diff < 0) {
      sec_diff = sec_diff - 1;
      nsec_diff = 1000000000 + nsec_diff;
    }
    return new ROSLIB.Time({
      secs: sec_diff,
      nsecs: nsec_diff
    });
  };

