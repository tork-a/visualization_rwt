$(function() {

  /* ROS */
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");

  var pub_speech = null;
  var publish = function(msg) {
    var name = $("#input-topic").val();
    if (!pub_speech || pub_speech.name !== name) { 
      pub_speech = new ROSLIB.Topic({
        ros: ros,
        name: name,
        messageType: 'speech_recognition_msgs/SpeechRecognitionCandidates'
      });
    }
    pub_speech.publish(msg);
  };

  var showInfo = function(id) {
    if (id) {
      $('#info').children().each(function(i, e) {
        if (e.style) {
          e.style.display = e.id == id ? 'inline' : 'none';
        }
      });
      $('#info').css('visibility', 'visible');
    } else {
      $('#info').css('visibility', 'hidden');
    }
  };

  showInfo("info_start");

  /* Speech Recognition */
  var sr = new RWTSpeechRecognition("select-language", "select-dialect");
  if (!sr.isSupported()) {
    showInfo("info_no_support");
    $("#button-start").attr('disabled', true);
  };

  var startStamp = null;
  sr.on("start", function() {
    showInfo("info_speak_now");
    $("#button-start").css("display", "none");
    $("#button-stop").css("display", "inline");
  });

  sr.on("error", function(event) {
    console.error(event);
    if (event.error == 'not-allowed') {
      if (event.timeStamp - startStamp < 100) {
        showInfo("info_blocked");
      } else {
        showInfo("info_denied");
      }
    }
    $("#button-start").css("display", "inline");
    $("#button-stop").css("display", "none");
  });

  sr.on("end", function() {
    showInfo("info_start");
    $("#button-start").css("display", "inline");
    $("#button-stop").css("display", "none");
  });

  sr.on("result", function(finals, interim) {
    if (interim) {
      $("#interim").html(interim);
    }
    
    if (finals && finals.length > 0) {
      $("#interim").html("");

      // show result as table
      var table = '<table class="table table-striped table-bordered table-condenced">'
      table += '<tr><td>#</td><td>Transcript</td><td>Confidence</td></tr>';
      for (var i = 0; i < finals.length; i++) {
        table += '<tr>';
        table += '<td>' + i + '</td>';
        table += '<td>' + finals[i].transcript + '</td>';
        table += '<td>' + finals[i].confidence + '</td>';
        table += '</tr>';
      }
      table += '</table>';
      $("#finals").prepend(table);

      // publish
      var msg = new ROSLIB.Message({
        transcript: finals.map(function(c) { return c.transcript; }),
        confidence: finals.map(function(c) { return c.confidence; })
      });
      publish(msg);
    }
  });

  $("#select-mode").change(function() {
    if ($("#select-mode").val() == "continuous") {
      sr.setContinuous(true);
    } else {
      sr.setContinuous(false);
    }
  });

  $("#button-start").on("click", function(e) {
    startStamp = e.timeStamp;
    sr.start();
  });

  $("#button-stop").on("click", function(e) {
    sr.stop();
  });
});
