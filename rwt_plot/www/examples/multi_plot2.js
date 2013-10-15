$(function() {
    var plot = new ROSLIB.RWTPlot({
        max_data: 100
    });

    plot.initializePlot("#plot-area", {
        yaxis: {
            max: 10000
        }
    });

    for (var i = 0; i < 101; i++) {
        if (i < 30) {
            plot.addData([i * i]);    // quadratic and linear
        }
        else if (i < 60){
            plot.addData([i * i, 10000 - i * i]);    // quadratic and linear
        }
        else {
            plot.addData([i * i]);    // quadratic and linear
        }
    }
    plot.draw();
});

