// Plot.js

/**
 * @fileOverview a file to define RWTPlot class.
 * @author Ryohei Ueda
 */


// class
/**
 * a class for plotting
 * @class RWTPlot
 * @param spec
 */
ROSLIB.RWTPlot = function(spec) {
  this.max_data = spec.max_data || 100; // defaults to 100
  this.use_timestamp = spec.timestamp;
  
  this.plot = null;
  this.drawingp = false;
  this.clearData();
};


ROSLIB.RWTPlot.prototype.clearData = function() {
  if (this.use_timestamp) {
    this.data = [];
  }
  else {
    this.data = new ROSLIB.RingBuffer({bufferCount: this.max_data});
  }
  this.need_to_animate = false;
  if (this.spec) {
    $(this.content).find('svg').remove();
    this.initializePlot(this.content, this.spec);
  }
};

ROSLIB.RWTPlot.prototype.initializePlot = function($content, spec) {
  this.spec = spec || {};
  this.content = $content;
  var width = $($content).width();
  var height = $($content).height();
  var margin = spec.margin || {top: 20, right: 20, bottom: 20, left: 40};
  var that = this;

  var yaxis_spec = spec.yaxis || {};
  var yaxis_min = yaxis_spec.min || 0.0;
  var yaxis_max = yaxis_spec.max || 1.0;

  this.y_autoscale = yaxis_spec.auto_scale || false;
  this.y_min_value = yaxis_min;
  this.y_max_value = yaxis_max;
  this.y_autoscale_margin = yaxis_spec.auto_scale_margin || 0.2;
  
  if (this.use_timestamp) {
    //this.x = d3.scale.linear().domain([0, this.max_data]).range([0, width - margin.left - margin.right]);
    this.x = d3.time.scale().range([0, width - margin.left - margin.right]);
    //.domain([0, this.max_data])
  }
  else {
    this.x = d3.scale.linear()
      .domain([0, this.max_data - 1])
      .range([0, width - margin.left - margin.right]);
  }
  if (this.y_autoscale) {
    this.y = d3.scale.linear()
      .domain(this.axisMinMaxWithMargin(this.y_min_value, this.y_max_value, this.y_autoscale_margin))
      .range([height - margin.top - margin.bottom, 0]);
  }
  else {
    this.y = d3.scale.linear()
    .domain([yaxis_min, yaxis_max])
    .range([height - margin.top - margin.bottom, 0]);
  }
  
  this.svg = d3.select($content).append('svg')
    .attr('class', 'rwt-plot')
    .attr('width', width)
    .attr('height', height)
    .append('g')
    .attr('transform', 'translate(' + margin.left + ',' + margin.top + ')');
  
  this.svg.append('defs').append('clipPath')
    .attr('id', 'clip')
    .append('rect')
    .attr('width', width - margin.left - margin.right)
    .attr('height', height - margin.top - margin.bottom);

  // draw x axis
  this.svg.append('g')
    .attr('class', 'x axis')
    .attr('transform', 'translate(0,' + this.y(0) + ')')
    .call(d3.svg.axis().scale(this.x).orient('bottom'));
  this.svg.append('g')
    .attr('class', 'y axis')
    .call(d3.svg.axis().scale(this.y).orient('left'));
  if (this.use_timestamp) {
    this.line = d3.svg.line()
      .x(function(d, i) { return that.x(d[0].toDate()); })
      .y(function(d, i) { return that.y(d[1]); });
  }
  else {
    this.line = d3.svg.line()
      .x(function(d, i) { return that.x(i); })
      .y(function(d, i) { return that.y(d); });
  }
  
  this.paths = [];
};

ROSLIB.RWTPlot.prototype.allocatePath = function(num) {
  return this.svg.append('g')
    .attr('clip-path', 'url(#clip)')
    .append('path')
    .datum([])
    .attr('class', 'line line' + num)
    .attr('d', this.line);
};

ROSLIB.RWTPlot.prototype.axisMinMaxWithMargin = function(min, max, margin) {
  var y_middle = (max + min) / 2.0;
  var y_min = (min - y_middle) * (1.0 + margin) + y_middle;
  var y_max = (max - y_middle) * (1.0 + margin) + y_middle;
  return [y_min, y_max];
};

ROSLIB.RWTPlot.prototype.checkYAxisMinMax = function(data) {
  var need_to_update = false;
  for (var dataIndex = 0; dataIndex < data.length; dataIndex++) {
    var val = data[dataIndex];
    if (val < this.y_min_value) {
      this.y_min_value = val;
      need_to_update = true;
    }
    if (val > this.y_max_value) {
      this.y_max_value = val;
      need_to_update = true;
    }
  }
  if (need_to_update) {
    this.y.domain(this.axisMinMaxWithMargin(this.y_min_value, this.y_max_value,
                                            this.y_autoscale_margin));
    this.svg.select('.y.axis')
      .call(d3.svg.axis().scale(this.y).orient('left'));
  }
};

ROSLIB.RWTPlot.prototype.addRawData = function(data) {
  // check the dimension
  var data_dimension = _.isArray(data) ? data.length : 0;
  if (data_dimension === 0) {
    data = [data];          // force to encapsulate into array
  }
  // check auto scale
  if (this.y_autoscale) {
    this.checkYAxisMinMax(data);
  }
  
  var now = ROSLIB.Time.now();
  this.data.push(data);
  var arr_data = this.data.toArray();
  var decomposed_arr = _.zip(arr_data);
  if (this.paths.length < data.length) {
    for (var pathIndex = this.paths.length; pathIndex < data.length; pathIndex++) {
      this.paths.push(this.allocatePath(pathIndex % 7));
    }
  }
  
  for (var i = 0; i < decomposed_arr.length; i++) {
    var target_arr = decomposed_arr[i];
    if (this.need_to_animate) {
      this.paths[i]
        .datum(target_arr)
        .attr('d', this.line)
        .attr('transform', null)
        .transition()
      //.duration(0)
        .ease('linear')
        .attr('transform', 'translate(' + this.x(-1) + ',0)');
    }
    else {
      this.paths[i].datum(target_arr)
        .attr('d', this.line)
        .attr('transform', null)
        .transition();
    }
  }
  // this.last_time = now;
  if (this.data.length() === this.max_data) {
    this.need_to_animate = true;
  }
};

ROSLIB.ROSTimeToSec = function(timea) {
  return timea.secs + timea.nsecs / 1000000000.0;
};

ROSLIB.ROSTimeDifference = function(timea, timeb) {
  return (timea.secs - timeb.secs) + (timea.nsecs - timeb.nsecs) / 1000000000.0;
};

ROSLIB.RWTPlot.prototype.chopTimestampedData = function(stamp, data) {
  // check the oldest message
  if (this.data.length > 0) {
    // chop here
    var chop_num = 0;
    for (var i = 0; i < this.data.length; i++) {
      //var diff = ROSLIB.ROSTimeDifference(stamp, this.data[i].stamp);
      var diff = stamp.substract(this.data[i].stamp).toSec();
      if (diff > this.max_data) {
        chop_num = chop_num + 1;
      }
      else {
        break;
      }
    }
    this.data = this.data.slice(chop_num);
    return chop_num > 0;
  }
  else {
    return false;
  }
};

ROSLIB.RWTPlot.prototype.addTimestampedData = function(stamp, data) {
  var data_dimension = _.isArray(data) ? data.length : 0;
  if (data_dimension === 0) {
    data = [data];          // force to encapsulate into array
  }
  if (this.y_autoscale) {
    this.checkYAxisMinMax(data);
  }
  
  if (this.paths.length < data.length) {
    for (var pathIndex = this.paths.length; pathIndex < data.length; pathIndex++) {
      this.paths.push(this.allocatePath(pathIndex % 7));
    }
  }
  var before_chop_oldest_time = null;
  if (this.data.length > 0) {
    before_chop_oldest_time = this.data[0].stamp;
  }
  var need_to_animate = this.chopTimestampedData(stamp, data);
  
  data.stamp = stamp;
  this.data.push(data);
  if (this.data.length < 1) {
    return;
  }
  var oldest_stamp = this.data[0].stamp;
  this.x.domain([oldest_stamp.toDate(), data.stamp.toDate()]);
  this.svg.select('.x.axis')
    .call(d3.svg.axis().scale(this.x).orient('bottom'));
  
  for (var i = 0; i < data.length; i++) { // x_i := i
    var plot_data = [];
    for (var j = 0; j < this.data.length; j++) {
      var value = this.data[j][i];
      // scale x(i) here
      // oldest_stamp = 0, stamp = this.max_data
      if (!stamp.equal(oldest_stamp)) {
        var x = this.data[j].stamp.substract(oldest_stamp).toSec();
        var new_data = [this.data[j].stamp, value]; // [x1, y1] or [x1, z1]
        //var new_data = [x, value]; // [x1, y1] or [x1, z1]
        plot_data.push(new_data);
      }
    }

    if (need_to_animate) {
      var translation = oldest_stamp.substract(before_chop_oldest_time).toSec();
      this.paths[i]
        .datum(plot_data)
        .attr('d', this.line)
        .attr('transform', null)
        .transition()
      //.duration(0)
        .ease('linear')
        .attr('transform', 'translate(' + (-translation) + ',0)');
    }
    else {
      this.paths[i].datum(plot_data)
        .attr('d', this.line)
        .attr('transform', null)
        .transition();
    }
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

// ROSLIB.RWTPlot.prototype.draw = function() {
//   var now = ROSLIB.Time.now();
//   if (this.plot) {
//     this.plot.setupGrid();
//     this.plot.draw();
//     this.last_draw_time = now;
//   }
// };
