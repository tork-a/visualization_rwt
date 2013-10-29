$(function() {
  var plot = new ROSLIB.RWTPlot({
    max_data: 10,          // when using timestamp, it is regarded as seconds
    timestamp: true
  });

  plot.initializePlot("#plot-area", {
    yaxis: {
      max: 300
    }
  });

  // subscribe topic
  var ros = new ROSLIB.Ros({
    url: "ws://" + location.hostname + ":8888"
  });
  var sub = new ROSLIB.Topic({
    ros: ros,
    name: "/random_point",
    messageType: "geometry_msgs/PointStamped"
  });
  sub.subscribe(function(msg) {
    plot.addData(ROSLIB.Time.fromROSMsg(msg.header.stamp), [msg.point.x, msg.point.y, msg.point.z])
  });
});
