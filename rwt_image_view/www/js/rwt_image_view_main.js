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
        $("#topic-select").append('<option value="' + topic + '">' + topic + "</option>");
      });
    });
  });

  ros.on("close", function() {
    $("#topic-select").empty();
  });

  
  ////// mouse point calc
  // generally, "canvas-area" size is not completelly equal to "canvas-area canvas", but we have to access "canvas-area" to get mouse position
  function calc_mouse_position_on_image(canvas, e){
    const clientRect = canvas.getBoundingClientRect();
    const canvasX    = e.clientX - clientRect.left;
    const canvasY    = e.clientY - clientRect.top;
    const scale      = current_image_info.height /  $("#canvas-area canvas").height();
    const imageX     = canvasX * scale;
    const imageY     = canvasY * scale;
    return {imageX, imageY};
  }
  
  ////// mouse move
  document.getElementById("canvas-area").addEventListener("mousemove", function(e){
    const {imageX, imageY} = calc_mouse_position_on_image(this, e);
    const point = new ROSLIB.Message({
      header : { frame_id : "test" },
      point  : { x : imageX, y : imageY, z : 0.0 }
    });
    const topic_name = current_image_info.topic + '/mousemove';
    const mousemove_pub = new ROSLIB.Topic({
      ros : ros,
      name : topic_name,
      messageType : 'geometry_msgs/PointStamped'
    });
    mousemove_pub.publish(point);
    document.getElementById("debug-text-area1").innerText = topic_name + " : " + imageX.toFixed(1) +", "+ imageY.toFixed(1);
  });

  ///// mouse click
  document.getElementById("canvas-area").addEventListener("click", function(e){
    const {imageX, imageY} = calc_mouse_position_on_image(this, e);
    const point = new ROSLIB.Message({
      header : { frame_id : "test" },
      point  : { x : imageX, y : imageY, z : 0.0 }
    });
    const topic_name = current_image_info.topic + '/screenpoint';
    const screenpoint_pub = new ROSLIB.Topic({
      ros : ros,
      name : topic_name,
      messageType : 'geometry_msgs/PointStamped'
    });
    screenpoint_pub.publish(point);
    document.getElementById("debug-text-area2").innerText = topic_name + " : " + imageX.toFixed(1) +", "+ imageY.toFixed(1);
  });

  ///// gazebo reset
  $("#reset-gazebo-button").click(function(e) {
    e.preventDefault();
    var resetWorld = new ROSLIB.Service({
      ros : ros,
      name : '/gazebo/reset_world',
      serviceType : 'std_srcs/Empty'
    });
    request = $.extend(true, {}, init_request);
    var request1 = new ROSLIB.ServiceRequest();
    resetWorld.callService(request1, result => { console.log('Call ' + setModelState.name); });
  });
  
  var mjpeg_canvas = null;
  var current_image_info = {topic:'', width:0, height:0, frame_id:''};
  $("#topic-form").submit(function(e) {
    if (mjpeg_canvas) {
      // remove the canvas here. (but mjpeg_canvas wonn't be released like this) 
      // https://github.com/tork-a/visualization_rwt/issues/92
      mjpeg_canvas = null;
      $("#canvas-area canvas").remove();
    }
    e.preventDefault();
    current_image_info.topic = $("#topic-select").val();
    // first of all, subscribe the topic and detect the width/height
    const image_sub_once = new ROSLIB.Topic({
      ros : ros,
      name : current_image_info.topic,
      messageType : 'sensor_msgs/Image'
    });
    image_sub_once.subscribe(message => {
      current_image_info.width = message.width;
      current_image_info.height = message.height;
      current_image_info.frame_id = message.header.frame_id;
      document.getElementById("debug-text-area3").innerText = current_image_info.topic + " (" + current_image_info.frame_id + "): "+ current_image_info.width +" x "+ current_image_info.height;
      // after receiving first image topic, set up correct aspect ratio mjpeg_canvas 
      if(!mjpeg_canvas){
    	const div_width = $("#canvas-area").width();
    	mjpeg_canvas = new MJPEGCANVAS.Viewer({
    	  divID : "canvas-area",
    	  host : ros.url().hostname,
    	  topic : current_image_info.topic,
    	  width: div_width,
    	  height: current_image_info.height * div_width / current_image_info.width
    	});
      }
      image_sub_once.unsubscribe();// subscribe only once to get image info (continuous subscription cause high traffic)
    });
    return false;
  });

  var recordingp = false;
  $("#record-button").click(function(e) {
    var $button = $(this);
    e.preventDefault();
    if (current_image_info.topic) {
      if (!recordingp) {
        var rosbagClient = new ROSLIB.Service({
          ros : ros,
          name : '/rosbag_record',
          serviceType : 'rwt_image_view/RosbagRecordRequest'
        });
        var request = new ROSLIB.ServiceRequest({
          topics: [current_image_info.topic]
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
