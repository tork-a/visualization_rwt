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

  
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");

  ros.on("connection", function() {
    ros.getTopicsForType('sensor_msgs/Image', function(image_topics) {
      // _.remove(image_topics, function(topic_name) {
      //     return topic_name.indexOf("prosilica") !== -1;
      // });
      image_topics.sort();
      var images_num = image_topics.length;
      //var $html = $('<div class="canvases"></div>');
      var html = "";
      for (var i = 0; i < images_num; i++) {
        var topic_name = image_topics[i];
        if (i % 3 === 0) {
          html = html + '<div class="row">';
        }
        html = html + '<div class="image-canvas col-md-4">' + topic_name + '<div id="image-' + i + '"></div></div>';
        
        if (i % 3 === 2) {
          html = html + '</div>'; // end of row
        }
      }
      if (images_num % 3 !== 2) {
        html = html + '</div>'; // end of row
      }
      
      // inserting to canvas-area
      $("#canvas-area").append(html);

      for (var i = 0; i < images_num; i++) {
        var divid = "image-" + i;
        mjpeg_canvas = new MJPEGCANVAS.Viewer({
          divID : divid,
          host : ros.url().hostname,
          topic : image_topics[i],
          width: $("#" + divid).width(),
          height: $("#" + divid).width() * 480 / 640.0
        });
      }
    });
  });

  ros.on("close", function() {
    $("#canvas-area").empty();
  });
  
  // var mjpeg_canvas = null;
  // var current_image_topic = null;
  // $("#topic-form").submit(function(e) {
  //   if (mjpeg_canvas) {
  //     // remove the canvas here
  //     mjpeg_canvas = null;
  //     $("#canvas-area canvas").remove();
  //   }
  //   e.preventDefault();
  //   var topic = $("#topic-select").val();
  //   // first of all, subscribe the topic and detect the width/height
  //   var div_width = $("#canvas-area").width();
  //   current_image_topic = topic;
  //   mjpeg_canvas = new MJPEGCANVAS.Viewer({
  //     divID : "canvas-area",
  //     host : location.hostname,
  //     topic : topic,
  //     width: div_width,
  //     height: 480 * div_width / 640.0
  //   });
  //   return false;
  // });

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
  
});
