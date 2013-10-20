$(function() {
    var plot = new ROSLIB.RWTPlot({
        max_data: 50
    });

    plot.initializePlot("#plot-area", {
        yaxis: {
            max: 100
        }
    });

    plot.draw();

    // subscribe topic
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":9090"
    });
    var sub = new ROSLIB.Topic({
            ros: ros,
      name: "/atlas/joint_states",
      messageType: "sensor_msgs/JointState"
            // name: "/random_float",
            // messageType: "std_msgs/Float64"
    });
    sub.subscribe(function(msg) {
      //plot.addData(msg.data);
      plot.addData(msg.position[10]);
        plot.draw();
    });
});
