$(function() {

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

  
  var ros = new ROSLIB.Ros({
    url: "ws://" + location.hostname + ":9090"
  });

  function extractImageTopics(topics, callback, prev_result) {
    if (!_.isArray(prev_result)) {
      prev_result = [];
    }
    if (topics.length === 0) {
      callback(prev_result);
    }
    else {
      isImageTopic(topics[0], function(result) {
        if (result) {
          extractImageTopics(topics.slice(1), callback, prev_result.concat([topics[0]]));
        }
        else {
          extractImageTopics(topics.slice(1), callback, prev_result);
        }
      });
    }
  }
  
  function isImageTopic(topic, callback) {
    ros.getTopicType(topic, function(type) {
      if (type === 'sensor_msgs/Image') {
        callback(true);
      }
      else {
        callback(false);
      }
    });
  };
  
  ros.getTopics(function(topics) {
    extractImageTopics(topics, function(image_topics) {
      // we need to extract sensor_msgs/Image topics
      $("#topic-select").append(_.map(image_topics, function(topic) {
        return '<option value="' + topic + '">' + topic + "</option>";
      }).join("\n"));
      $("#topic-select").change();
    });
  });

  var mjpeg_canvas = null;
  $("#topic-form").submit(function(e) {
    if (mjpeg_canvas) {
      // remove the canvas here
      mjpeg_canvas = null;
      $("#canvas-area canvas").remove();
    }
    e.preventDefault();
    var topic = $("#topic-select").val();
    // first of all, subscribe the topic and detect the width/height
    var div_width = $("#canvas-area").width();
    mjpeg_canvas = new MJPEGCANVAS.Viewer({
      divID : "canvas-area",
      host : location.hostname,
      topic : topic,
      width: div_width,
      height: 480 * div_width / 640.0
    });
    return false;
  });
  
});