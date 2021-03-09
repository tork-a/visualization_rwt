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

  ros.on("connection", function() {
    ros.getTopicsForType('sensor_msgs/Image', function(image_topics) {
      image_topics.sort();
      _.map(image_topics, function(topic) {
        $("#image-topic-select").append('<option value="' + topic + '">' + topic + "</option>");
      });
    });
    ros.getTopicsForType('geometry_msgs/Twist', function(joy_topics) {
      joy_topics.sort();
      _.map(joy_topics, function(topic) {
        $("#joy-topic-select").append('<option value="' + topic + '">' + topic + "</option>");
      });
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

  var cmdVel = null;
  $("#joy-topic-form").submit(function(e) {
    if (mjpeg_canvas) {
      // remove the canvas here
      mjpeg_canvas = null;
      $("#canvas-area canvas").remove();
    }
    e.preventDefault();
    var topic = $("#joy-topic-select").val();
    cmdVel = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/Twist'
    });
    return false;
  });

  var manager = null;
  var lin = 0.0;
  var ang = 0.0;
  var linvel = 0.5;
  var angvel = 1.0;

  var twist = new ROSLIB.Message({
    linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    }
  });

  if (manager == null) {
    joystickContainer = document.getElementById('joystick');
    var options = {
      zone: joystickContainer,
      color: "#6000b5",
      size: 500,
      mode: 'static',
      restJoystick: true
    };
    manager = nipplejs.create(options);
    manager.on('move', function (evt, data) {
      try {
          var dist = data.distance;
          var angle = data.angle.degree;
      }
      catch(error){
          createJoystick();
          console.error(error);
      }
      lin = Math.sin(angle / 57.29) * dist / 100.0 * linvel;
      ang = Math.cos(angle / 57.29) * dist / 100.0 * -angvel;
      move();
    });
    manager.on('end', function () {
      lin = 0.0;
      ang = 0.0;
      move();
    });
  }

  function move() {
    if (cmdVel) {
      if (lin !== undefined && ang !== undefined) {
        twist.linear.x = lin;
        twist.angular.z = ang;
      } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
      }
      cmdVel.publish(twist);
    }
  }

  var bl1 = document.getElementById("bl1");
  var bl2 = document.getElementById("bl2");
  var ba1 = document.getElementById("ba1");
  var ba2 = document.getElementById("ba2");

  var lindisp= document.getElementById("lindisp");
  var angdisp= document.getElementById("angdisp");

  bl1.addEventListener("click", function() {
    var next_linvel = linvel + 0.1;
    if (next_linvel >= 0) {
      linvel = next_linvel;
      lindisp.textContent = linvel;
    }
  });
  bl2.addEventListener("click", function() {
    var next_linvel = linvel - 0.1;
    if (next_linvel >= 0) {
      linvel = next_linvel;
      lindisp.textContent = linvel;
    }
  });
  ba1.addEventListener("click", function() {
    var next_angvel = angvel + 0.1;
    if (next_angvel >= 0) {
      angvel = next_angvel;
      angdisp.textContent = angvel;
    }
  });
  ba2.addEventListener("click", function() {
    var next_angvel = angvel - 0.1;
    if (next_angvel >= 0) {
      angvel = next_angvel;
      angdisp.textContent = angvel;
    }
  });
});
