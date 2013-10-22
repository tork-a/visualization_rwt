$(function() {
  var plot = new ROSLIB.RWTPlot({
    max_data: 50
  });

  plot.initializePlot("#plot-area", {
    yaxis: {
      max: 1.0
    }
  });

  // subscribe topic
  var ros = new ROSLIB.Ros({
    url: "ws://" + location.hostname + ":8888"
  });
  var sub = new ROSLIB.Topic({
    ros: ros,
    name: "/random_float",
    messageType: "std_msgs/Float64"
  });
  sub.subscribe(function(msg) {
    plot.addData(msg.data);
  });
});
