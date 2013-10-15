$(function() {
    var plot = new ROSLIB.RWTPlot({
        max_data: 100
    });
    plot.initializePlot("#plot-area", {
        yaxis: {
            max: 10000
        }
    });
    for (var i = 0; i < 101; i++)
        plot.addData(i * i);    // quadratic
    plot.draw();
});
