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
        default_option = "/kinect_head/rgb/image_rect_color";
        $("#topic-select").append('<option value="' + topic + (topic == default_option ? '" selected>' : '">') + topic + "</option>");
      });
    });
    ros.getTopicsForType('audio_common_msgs/AudioData', function(audio_topics) {
      audio_topics.sort();
      _.map(audio_topics, function(topic) {
        $("#audio-select").append('<option value="' + topic + '">' + topic + "</option>");
      });
    });
  });

  ros.on("close", function() {
    $("#topic-select").empty();
    $("#audio-select").empty();
  });

  ///// servo state
  new ROSLIB.Topic({
    ros : ros,
    name : '/pr2_ethercat/motors_halted',
    messageType : 'std_msgs/Bool'
  }).subscribe(message => {
    isOn = (message.data == false); // (halted == false) = servoOn
    txtarea = document.getElementById("servo-state-text");
    txtarea.innerText             = (isOn ? "Servo ON" : "Servo OFF");
    txtarea.style.backgroundColor = (isOn ? "green" : "gray");
    txtarea.style.color           = (isOn ? "white" : "red");
  });

  ///// LR click mode
  new ROSLIB.Topic({
    ros : ros,
    name : '/rwt_current_state',
    messageType : 'std_msgs/String'
  }).subscribe(message => {
    isL = (message.data == "l");
    ltxtarea = document.getElementById("l-text");
    ltxtarea.innerText             = (isL  ? "L Arm Click Enable" : "L Arm Click Disable");
    ltxtarea.style.backgroundColor = (isL  ? "green" : "gray");
    ltxtarea.style.color           = (isL  ? "white" : "red");
    rtxtarea = document.getElementById("r-text");
    rtxtarea.innerText             = (!isL ? "R Arm Click Enable" : "R Arm Click Disable");
    rtxtarea.style.backgroundColor = (!isL ? "green" : "gray");
    rtxtarea.style.color           = (!isL ? "white" : "red");
  });

  ///// search all buttons in "rwt-command-buttons" div and set callback
  const rwt_command_pub = new ROSLIB.Topic({
    ros : ros,
    name : "/rwt_command_string",
    messageType : 'std_msgs/String'
  });
  buttons = Array.from(document.getElementById("rwt-command-buttons").getElementsByTagName("button"));
  buttons.forEach(
    function(b){
      b.onclick = function(){
        rwt_command_pub.publish( new ROSLIB.Message({ data : b.id }) ); // simply send button id as rwt command string
      }
    }
  )
  
 

  ///// calc round trip delay for individual client
  const client_id = "random" + Math.random().toString(32).substring(2);
  const time_echo_topic = new ROSLIB.Topic({ // used both in pub/sub
    ros : ros,
    name : '/time_echo_' + client_id,
    messageType : 'std_msgs/Time'
  });
  var calc_delay = function(){ // pub
    const time_now = Date.now();// msec
    time_echo_topic.publish(
      new ROSLIB.Message({
        data : { secs : Math.floor(time_now / 1e3), nsecs : (time_now % 1e3) * 1e6}
      })
    );
  } 
  setInterval(calc_delay, 100);//100ms
  time_echo_topic.subscribe(message => { // sub
    const time_now = message.data.secs * 1e3 + message.data.nsecs / 1e6;
    document.getElementById("debug-text-area4").innerText = "Delay: " + (Date.now() - time_now) + " [ms] (client id: " + client_id + ")";
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

  ///// 3D point
  listener1 = new ROSLIB.Topic({
    ros : ros,
    name : '/pointcloud_screenpoint_nodelet/output_point',
    messageType : 'geometry_msgs/PointStamped'
  });
  listener1.subscribe(message => {
    document.getElementById("debug-text-area3").innerText =
      listener1.name + " = "+ message.point.x +", "+ message.point.y +", "+ message.point.z +"\n\n";
  });

  var audio_pub = new ROSLIB.Topic({
    ros : ros,
    name : '/audio_from_' + client_id + '/audio',
    messageType : 'audio_common_msgs/AudioData'
  });

  let audioData = [];
  const bufferSize = 16384;//max

  const Float32ArrayTo16BitPCM = function (samples) {
    let buffer = new ArrayBuffer(samples.length * 2);
    let view = new DataView(buffer);
    const floatTo16BitPCM = function (output, input) {
      for (let i = 0; i < input.length; i++) {
        let s = Math.max(-1, Math.min(1, input[i]));
        output.setInt16(i*2, s < 0 ? s * 0x8000 : s * 0x7FFF, true);
      }
    };
    floatTo16BitPCM(view, samples); // 波形データ
    return view;
  };

  // save audio data
  const onAudioProcess = function (e) {
    document.getElementById("debug-text-area6").innerText = "Your Mic is ON ("+ audio_pub.name +")";
    const input = e.inputBuffer.getChannelData(0);
    const dataview = Float32ArrayTo16BitPCM(input);
    const base64String = btoa(String.fromCharCode(...new Uint8Array(dataview.buffer)));
    audio_pub.publish( new ROSLIB.Message({ data : base64String }) );
  };

  // getusermedia
  const handleSuccess = function (stream) {
    document.getElementById("debug-text-area6").innerText = "navigator.mediaDevices.getUserMedia() success";
    let audioContext = new AudioContext({
      latencyHint: 'interactive',
      sampleRate: 16000,
    });
    var buf = audioContext.createBuffer(1, bufferSize, audioContext.sampleRate);
    scriptProcessor = audioContext.createScriptProcessor(bufferSize, 1, 1);
    var mediastreamsource = audioContext.createMediaStreamSource(stream);
    mediastreamsource.connect(scriptProcessor);
    scriptProcessor.onaudioprocess = onAudioProcess;
    scriptProcessor.connect(audioContext.destination);
    console.log('setup audio complete. samplerate: ' + audioContext.sampleRate + " bufferSize: " + bufferSize);
  };

  if(navigator.mediaDevices){
    navigator.mediaDevices.getUserMedia({ audio: true, video: false })
      .then(handleSuccess)
      .catch(function(err){
        document.getElementById("debug-text-area6").innerText = "Cannot get Mic device";
        console.log('getUserMedia fail');
      });
  }else{
    console.log('navigator.mediaDevices not found');
    document.getElementById("debug-text-area6").innerText = "navigator.mediaDevices not found";
  }

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
        quality : 50,
    	  width: div_width,
    	  height: current_image_info.height * div_width / current_image_info.width
    	});
      }
      image_sub_once.unsubscribe();// subscribe only once to get image info (continuous subscription cause high traffic)
    });
    return false;
  });

  $("#audio-topic-update-button").click(function(e) {
    $("#audio-select").empty();
    ros.getTopicsForType('audio_common_msgs/AudioData', function(audio_topics) {
      audio_topics.sort();
      _.map(audio_topics, function(topic) {
        $("#audio-select").append('<option value="' + topic + '">' + topic + "</option>");
      });
    });
  });

  let audio_sub = null;
  var current_audio_info = {topic:''};
  $("#audio-form").submit(function(e) {
    if(audio_sub){
      audio_sub.unsubscribe();
    }
    current_audio_info.topic = $("#audio-select").val();
    var base64;
    audio_sub = new ROSLIB.Topic({
      ros : ros,
      name : current_audio_info.topic,
      messageType : 'audio_common_msgs/AudioData'
    });
    audio_sub.subscribe(message => {
      document.getElementById("debug-text-area5").innerText = audio_sub.name + " = "+ message.data +"\n\n";
      base64 = message.data;
      B=(f,b=4)=>String.fromCodePoint.apply(this, Array(b).fill().map((v,i)=>(f>>i*8)&255))
      D=f=>B(f,2)
      // tweek this:
      samplerate=16000 // keep this above 4000 hz
      ch=1 // channels
      bits=16 // multiples of 8
      samples=samplerate,
      s1=16,
      s2=samples*ch*bits/8
      header="RIFF"+
      B(4+(8+s1)+(8+s2))+ "WAVEfmt "+ //chunksize
      B(s1)+ //subchunk1size
      D(1)+ //format
      D(ch)+ //channels
      B(samplerate)+
      B(samplerate*ch*bits/8)+ //Byte rate
      D(ch*bits/8)+ // align
      D(bits)+"data"+ //8 Bits per sample
      D(s2) // subchunk2size
      h64=window.btoa(header)
      var sound = new Audio("data:audio/wav;base64," + h64 + base64);
      sound.play();
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
