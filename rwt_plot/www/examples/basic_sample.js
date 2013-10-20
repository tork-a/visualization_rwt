// $(function() {
    var plot = new ROSLIB.RWTPlot({
        max_data: 100
    });
    plot.initializePlot("#plot-area", {
        yaxis: {
            max: 1.0
        }
    });
    for (var i = 0; i < 101; i++)
        plot.addData(i * i / 100 / 100);    // quadratic
    plot.draw();
// });
