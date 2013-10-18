
$(function() {
  
  var plot = new ROSLIB.RWTPlot({
    max_data: 3,          // when using timestamp, it is regarded as seconds
    timestamp: true
  });
  
  plot.initializePlot("#plot-area", {
    interaction: {
      redrawOverlayInterval: 100000 / 60
    },
    yaxis: {
      auto_scale: true
    }
  });

  plot.draw();

  function getValFromAccessor(msg, accessor) {
    if (accessor.length == 0) {
      return msg;
    }
    else {
      if (accessor[0].match(/\[[\d]+\]/)) {
        var array_index = parseInt(accessor[0].match(/\[([\d]+)\]/)[1], 10);
        return getValFromAccessor(msg[accessor[0].split("[")[0]][array_index], accessor.slice(1));
      }
      else {
        return getValFromAccessor(msg[accessor[0]], accessor.slice(1));
      }
    }
  };
  
  // subscribe topic
  var ros = new ROSLIB.Ros({
    url: "ws://" + location.hostname + ":9090"
  });
  
  var sub = null;
  $("#topic-form").submit(function(e) {
    e.preventDefault();
    var topic_name = $("#topic-select").val();
    var accessor = $("#field-accessor").val().split("/");
    
    if (sub) {
      console.log("unsubscribe");
      sub.unsubscribe();
    }
    ros.getTopicType(topic_name, function(topic_type) {
      sub = new ROSLIB.Topic({
        ros: ros,
        name: topic_name,
        messageType: topic_type
      });
      plot.clearData();
      sub.subscribe(function(msg) {
        plot.addData(ROSLIB.Time.now(),
                     getValFromAccessor(msg, accessor));
        plot.draw();
      });
    });
    return false;
  });
  
  ros.getTopics(function(topics) {
    $("#topic-select").append(_.map(topics, function(topic) {
      return '<option value="' + topic + '">' + topic + "</option>";
    }).join("\n"));
    $("#topic-select").change();
  });
  $("#topic-select").change(function() {
    var topic_name = $("#topic-select").val();
    ros.getTopicType(topic_name, function(topic_type) {
      ros.getMessageDetails(topic_type, function(details) {
        var decoded = ros.decodeTypeDefs(details);
        $("#message-detail").find("pre").html(JSON.stringify(decoded, null, "  ")); // pretty print
      });
    });
  });
});
