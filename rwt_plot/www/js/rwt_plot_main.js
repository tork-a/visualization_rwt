
$(function() {
  
  var plot = new ROSLIB.RWTPlot({
    max_data: 2,          // when using timestamp, it is regarded as seconds
    timestamp: true
  });
  
  plot.initializePlot($("#plot-area"), {
    yaxis: {
      auto_scale: true,
      auto_scale_margin: 0.2,
      min: 0.1
    }
  });

  function getValFromAccessor(msg, accessor) {
    if (accessor.length == 0) {
      return msg;
    }
    else {
      if (_.isArray(accessor[0])) {
        return getValFromAccessor(msg[accessor[0][0]][accessor[0][1]], accessor.slice(1));
      }
      else {
        return getValFromAccessor(msg[accessor[0]], accessor.slice(1));
      }
    }
  };
  
  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");
  
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
      var i = 0;
      var prev = ROSLIB.Time.now();
      accessor = _.map(accessor, function(a) {
        if (a.match(/\[[\d]+\]/)) {
          var array_index = parseInt(a.match(/\[([\d]+)\]/)[1], 10);
          return [a.split("[")[0], array_index];
        }
        else {
          return a;
        }
      });
      sub.subscribe(function(msg) {
        var now = null;
        if (msg.header && msg.header.stamp) {
          now = ROSLIB.Time.fromROSMsg(msg.header.stamp);
        }
        else {
          now = ROSLIB.Time.now();
        }
        
        //plot.addData(getValFromAccessor(msg, accessor));
        plot.addData(now,
                     getValFromAccessor(msg, accessor));
        
        var diff = now.substract(prev);
        //if (diff.toSec() > 1.0) {
        //console.log('sec: ' + diff.toSec());
        //}
        prev = now;
      });
    });
    return false;
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

  ros.on("connection", function() {
    ros.getTopics(function(topic_info) {
      var topics = topic_info.topics;
      topics.sort();
      $("#topic-select").append(_.map(topics, function(topic) {
        return '<option value="' + topic + '">' + topic + "</option>";
      }).join("\n"));
      $("#topic-select").change();
    });
  });

  ros.on("close", function() {
    $("#topic-select").empty();
  });
});
