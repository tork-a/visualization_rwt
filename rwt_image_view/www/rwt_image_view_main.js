$(function() {

  ROSLIB.Ros.prototype.getTopicsForType = function(type, callback) {
    var topicsForTypeClient = new ROSLIB.Service({
      ros : this,
      name : '/rosapi/topics_for_type',
      serviceType : 'rosapi/TopicsForType'
    });
    var request = new ROSLIB.ServiceRequest({
      type: type
    });
    topicsForTypeClient.callService(request, function(result) {
      callback(result.topics);
    });
  };
  
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
    url: "ws://" + location.hostname + ":8888"
  });

  ros.getTopicsForType('sensor_msgs/Image', function(image_topics) {
    image_topics.sort();
    _.map(image_topics, function(topic) {
      $("#topic-select").append('<option value="' + topic + '">' + topic + "</option>");
    });
  });

  var mjpeg_canvas = null;
  var current_image_topic = null;
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
    current_image_topic = topic;
    mjpeg_canvas = new MJPEGCANVAS.Viewer({
      divID : "canvas-area",
      host : location.hostname,
      topic : topic,
      width: div_width,
      height: 480 * div_width / 640.0
    });
    $("#canvas-area canvas").css({ "position":"absolute",
                                   "left": "0px",
                                   "top": "0px",
                                   "z-index": "1"});
    return false;
  });

  var recordingp = false;
  $("#record-button").click(function(e) {
    var $button = $(this);
    e.preventDefault();
    if (current_image_topic) {
      if (!recordingp) {
        var rosbagClient = new ROSLIB.Service({
          ros : ros,
          name : '/rosbag_record',
          serviceType : 'rwt_image_view/RosbagRecordRequest'
        });
        var request = new ROSLIB.ServiceRequest({
          topics: [current_image_topic]
        });
        rosbagClient.callService(request, function(result) {
          recordingp = true;
          $button.removeClass("btn-success")
            .addClass("btn-danger")
            .html("stop recording");
          // download
          
        });
      }
      else {
        recordingp = false;
        var rosbagClient = new ROSLIB.Service({
          ros : ros,
          name : '/rosbag_record_stop',
          serviceType : 'std_srvs/Empty'
        });
        var request = new ROSLIB.ServiceRequest({});
        rosbagClient.callService(request, function(result) {
          // download here
          //var $alert = $('');
          var html = '<div class="alert alert-info alert-dismissable" id="download-alert">\
          <button type="button" class="close" data-dismiss="alert" aria-hidden="true">&times;</button>\
          <a class="alert-link" href="/rwt_image_view/tmp.bag">download the bagfile from here via right-click</a>\
</div>';
          //$button.html('<a href="/rwt_image_view/tmp.bag">download</a>');
          $("#topic-area").before(html);
          $button.removeClass("btn-danger")
            .addClass("btn-success")
            .html("record");
        });
      }
    }
  });

  var overlay_canvas = null;
  var overlay_context = null;
  var pencilmodep = false;
  $("#pencil-button").click(function (){
    if (pencilmodep){
      if (overlay_canvas) overlay_canvas = null;
      $("#canvas-area #overlay-canvas").remove();
      pencilmodep = false;
    } else {
      overlay_canvas = document.createElement('canvas');
      overlay_canvas.width = mjpeg_canvas.width;
      overlay_canvas.height = mjpeg_canvas.height;
      $("#canvas-area #overlay-canvas").css({ "z-index": "2",
                                              "position": "absolute",
                                              "left": "0px",
                                              "top": "0px"});
      $("#canvas-area #overlay-canvas").append(overlay_canvas);
      overlay_context = overlay_canvas.getContext('2d');
      pencilmodep = true;
    }
  });

  $.fn.drawTouch = function(){
    var start = function(e) {
      var touchEvent = e.originalEvent.changedTouches[0];
      overlay_context.beginPath();
      overlay_context.moveTo(touchEvent.pageX, touchEvent.pageY);
      };
    var move = function(e) {
      var touchEvent = e.originalEvent.changedTouches[0];
      e.preventDefault();
      overlay_context.lineTo(touchEvent.pageX, touchEvent.pageY);
      overlay_context.stroke();
    };
    $(this).touchstart(start);
    $(this).touchmove(move);
  };

  $.fn.drawMouse = function() {
    var clickp = false;
    var start = function(e) {
      clickp = true;
      overlay_context.beginPath();
      overlay_context.moveTo(e.pageX, e.pageY);
    };
    var move = function(e) {
      if(clickp){
        overlay_context.lineTo(e.pageX, e.pageY);
        overlay_context.stroke();
      }
    };
    var stop = function(e) {
      clickp = false;
    };
    $(this).mousedown(start);
    $(this).mousemove(move);
    $(this).mouseup(stop);
  };

});
