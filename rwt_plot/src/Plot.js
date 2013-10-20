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
};

ROSLIB.RWTPlot.prototype.initializePlot = function($content, spec, data) {
  var width = $($content).width();
  var height = $($content).height();
  var margin = {top: 20, right: 20, bottom: 20, left: 40};
  var that = this;

  var yaxis_spec = spec.yaxis || {};
  var yaxis_min = yaxis_spec.min || 0.0;
  var yaxis_max = yaxis_spec.max || 1.0;
  
  this.x = d3.scale.linear()
    .domain([0, this.max_data - 1])
    .range([0, width - margin.left - margin.right]);
  
  this.y = d3.scale.linear()
    .domain([yaxis_min, yaxis_max])
    .range([height - margin.top - margin.bottom, 0]);

  
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
  this.line = d3.svg.line()
    .x(function(d, i) { return that.x(i); })
    .y(function(d, i) { return that.y(d); });
  
  this.paths = [];
  this.arr_data = [];
};

ROSLIB.RWTPlot.prototype.allocatePath = function(num) {
  return this.svg.append('g')
    .attr('clip-path', 'url(#clip)')
    .append('path')
    .datum([])
    .attr('class', 'line line' + num)
    .attr('d', this.line);
};

ROSLIB.RWTPlot.prototype.addRawData = function(data) {
  // check the dimension
  var data_dimension = _.isArray(data) ? data.length : 1;
  if (data_dimension === 1) {
    data = [data];          // force to encapsulate into array
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
  // var plot_data = [];
  // // plot_data := [[[x1, y1], [x1, y2], [x1, y3], ...], [[x1, z1], [x1, z2], ...], ...]
  // for (var i = 0; i < this.data.length(); i++) { // x_i := i
  //   var arr = this.data.toArray();
  //   for (var j = 0; j < arr[i].length; j++) {
  //     var value = arr[i][j];
  //     var new_data = [i, value]; // [x1, y1] or [x1, z1]
  //     if (plot_data.length <= j) {
  //       // adding new empty array to plot_data
  //       plot_data.push([]);
  //     }
  //     plot_data[j].push(new_data);
  //   }
  // }
  // if (this.plot) {
  //   this.plot.setData(plot_data);
  // }
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
};

ROSLIB.RWTPlot.prototype.addTimestampedData = function(stamp, data) {
  this.chopTimestampedData(stamp, data);
  var data_dimension = _.isArray(data) ? data.length : 1;
  if (data_dimension === 1) {
    data = [data];          // force to encapsulate into array
  }
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
      if (!stamp.equal(oldest_stamp)) {
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

// ROSLIB.RWTPlot.prototype.draw = function() {
//   var now = ROSLIB.Time.now();
//   if (this.plot) {
//     this.plot.setupGrid();
//     this.plot.draw();
//     this.last_draw_time = now;
//   }
// };
