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
  ros.install_config_button("config-button", true, 9090);

  var map_div_width = $("#map").width();
  var viewer = new ROS2D.Viewer({
      divID : 'map',
      width: map_div_width,
      height: 480 * map_div_width / 640.0,
      background : '#FFFFFF'
  });

  var gridClient = new NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer.scene,
      viewer : viewer,
      serverName : '/move_base',
      withOrientation : true
  });

  ros.on("connection", function() {
    ros.getTopicsForType('sensor_msgs/Image', function(image_topics) {
      image_topics.sort();
      _.map(image_topics, function(topic) {
        $("#image-topic-select").append('<option value="' + topic + '">' + topic + "</option>");
      });
    });

    var mouseDown = false;
    var zoomKey = false;
    var panKey = false;
    var startPos = new ROSLIB.Vector3();
 
    var zoomView = new ROS2D.ZoomView({
        rootObject : viewer.scene
    });

    var panView = new ROS2D.PanView({
        rootObject : viewer.scene
    });
 
    viewer.scene.addEventListener('stagemousedown', function(event) {
      console.log("event");
      if (event.nativeEvent.ctrlKey === true) {
        zoomKey = true;
        zoomView.startZoom(event.stageX, event.stageY);
      }
      else if (event.nativeEvent.shiftKey === true) {
        panKey = true;
        panView.startPan(event.stageX, event.stageY);
      }
      startPos.x = event.stageX;
      startPos.y = event.stageY;
      mouseDown = true;
    });
    viewer.scene.addEventListener('stagemousemove', function(event) {
      if (mouseDown === true) {
        if (zoomKey === true) {
          var dy = event.stageY - startPos.y;
          var zoom = 1 + 10*Math.abs(dy) / viewer.scene.canvas.clientHeight;
          if (dy < 0)
              zoom = 1 / zoom;
          zoomView.zoom(zoom);
        }
        else if (panKey === true) {
            panView.pan(event.stageX, event.stageY);
        }
      }
    });
    viewer.scene.addEventListener('stagemouseup', function(event) {
      if (mouseDown === true) {
        zoomKey = false;
        panKey = false;
        mouseDown = false;
      }
    });
  });

  ros.on("close", function() {
    $("#image-topic-select").empty();
  });

  var mjpeg_canvas = null;
  var current_image_topic = null;
  $("#image-topic-form").submit(function(e) {
    if (mjpeg_canvas) {
      // remove the canvas here
      mjpeg_canvas = null;
      $("#canvas-area canvas").remove();
    }
    e.preventDefault();
    var topic = $("#image-topic-select").val();
    // first of all, subscribe the topic and detect the width/height
    var div_width = $("#canvas-area").width();
    current_image_topic = topic;
    mjpeg_canvas = new MJPEGCANVAS.Viewer({
      divID : "canvas-area",
      host : ros.url().hostname,
      topic : topic,
      width: div_width,
      height: 480 * div_width / 640.0
    });
    return false;
  });
});
