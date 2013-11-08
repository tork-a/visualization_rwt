// RoslibExt.js

/**
 * @fileOverview a file to extend ROSLIB.
 *  these are should be merged into roslibjs.
 * @author Ryohei Ueda
 */

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


/**
 * Represents a time whith seconds and nanoseconds
 * @class Time
 * @param spec - a dictionary which include nsecs and secs as the keys.
 * @property secs {Integer} seconds
 * @property nsecscs {Integer} nanoseconds (10^-9)
 */
ROSLIB.Time = function(spec) {
  this.nsecs = Math.floor((spec || {}).nsecs || 0);
  this.secs = Math.floor((spec || {}).secs || 0);
};

ROSLIB.Time.now = function() {
  var now = new Date();
  var msec = now.getTime();
  return new ROSLIB.Time({
    secs: Math.floor(msec / 1000),
    nsecs: (msec % 1000) * 1000000
  });
};

ROSLIB.Time.prototype.toSec = function() {
  return this.secs + this.nsecs / 1000000000.0;
};

ROSLIB.Time.prototype.toMillSec = function() {
  return this.secs * 1000 + this.nsecs / 1000000.0;
};

ROSLIB.Time.prototype.add = function(another) {
  var sec_added = this.secs + another.secs;
  var nsec_added = this.nsecs + another.nsecs;
  if (nsec_added > 1000000000) {
    sec_added = sec_added + 1;
    nsec_added = nsec_added - 1000000000;
  }
  return new ROSLIB.Time({
    secs: sec_added,
    nsecs: nsec_added
  });
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

ROSLIB.Time.prototype.equal = function(another) {
  var diff = this.substract(another);
  return ((diff.secs === 0) && (diff.nsecs === 0));
};

/**
 * Converts a JSON-ized message of stamp into ROSLIB.Time
 * @param msg - a message of stamp.
 */
ROSLIB.Time.fromROSMsg = function(msg) {
  return new ROSLIB.Time({secs: msg.secs, nsecs: msg.nsecs});
};

ROSLIB.Time.fromSec = function(sec) {
  return new ROSLIB.Time({secs: sec, nsecs: 0});
};

/**
 * Converts ROSLIB.Time into Date object
 */
ROSLIB.Time.prototype.toDate = function() {
  var d = new Date();
  d.setTime(this.secs * 1000 + this.nsecs / 1000000);
  return d;
};
