/**
 * @fileOverview a file to define RingBuffer class
 * @author Ryohei Ueda
 */

/**
 * a class for ring buffer.
 * @class RingBuffer
 * @param spec
 * @property bufferCount
 * 
 */
ROSLIB.RingBuffer = function(spec) {
  this.bufferCount = (spec || {}).bufferCount || 100;
  this.clear();
};

ROSLIB.RingBuffer.prototype.push = function(data) {
  this.endIndex++;
  if (this.endIndex === this.bufferCount) {
    this.endIndex = 0;
  }
  if (this.count >= this.bufferCount) {
    this.buffer[this.endIndex] = data;
    // increment startIndex and endIndex
    this.startIndex++;
    if (this.startIndex === this.bufferCount) {
      this.startIndex = 0;
    }
  }
  else {                        // not filled yet
    this.buffer[this.endIndex] = data;
  }
  this.count++;
  return data;
};

ROSLIB.RingBuffer.prototype.clear = function() {
  this.buffer = new Array(this.bufferCount);
  this.startIndex = 0;
  this.endIndex = -1;
  this.count = 0;
};


ROSLIB.RingBuffer.prototype.map = function(proc) {
  var ret = [];
  for (var i = this.startIndex; i < Math.min(this.count, this.bufferCount); i++) {
    ret.push(proc.call(this, this.buffer[i]));
  }
  if (this.count > this.bufferCount) {
    for (var j = 0; j < this.endIndex + 1; j++) {
      ret.push(proc.call(this, this.buffer[j]));
    }
  }
  return ret;
};

ROSLIB.RingBuffer.prototype.toArray = function() {
  return this.map(function(x) { return x; });
};

ROSLIB.RingBuffer.prototype.length = function() {
  return Math.min(this.bufferCount, this.count);
};