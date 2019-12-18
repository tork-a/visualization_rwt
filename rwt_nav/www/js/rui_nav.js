var ros;

function rosconnection() {
    // Connecting to ROS
     ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });
         
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
    });
         
    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });
         
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });
    
}

function nav() {
    var viewer = new ROS2D.Viewer({
        divID : 'map',
        width : 1000,
        height : 500,
	    background : '#FFFFFF'
    });

    var gridClient = new NAV2D.OccupancyGridClientNav({
        ros : ros,
        rootObject : viewer.scene,
        viewer : viewer,
        serverName : '/move_base',
        withOrientation : true
    });

    var zoomView = new ROS2D.ZoomView({
        rootObject : viewer.scene
    });
            
    var panView = new ROS2D.PanView({
        rootObject : viewer.scene
    });
           
        function registerMouseHandlers() {
                
            var mouseDown = false;
            var zoomKey = false;
            var panKey = false;
            var startPos = new ROSLIB.Vector3();
            
            viewer.scene.addEventListener('stagemousedown', function(event) {
                console.log("event");
                if (event.nativeEvent.ctrlKey === true) {
                    zoomKey = true;
                    zoomView.startZoom(event.stageX, event.stageY);
                }
                else if (event.nativeEvent.shiftKey === true) {
                    panKey = true;
                    panView.startPan(event.stageX, event.stageY);
                }
                startPos.x = event.stageX;
                startPos.y = event.stageY;
                mouseDown = true;
            });
            
            viewer.scene.addEventListener('stagemousemove', function(event) {
                if (mouseDown === true) {
                    if (zoomKey === true) {
                        var dy = event.stageY - startPos.y;
                        var zoom = 1 + 10*Math.abs(dy) / viewer.scene.canvas.clientHeight;
                        if (dy < 0)
                            zoom = 1 / zoom;
                        zoomView.zoom(zoom);
                    }
                    else if (panKey === true) {
                        panView.pan(event.stageX, event.stageY);
                    }
                }
            });
    
            viewer.scene.addEventListener('stagemouseup', function(event) {
                if (mouseDown === true) {
                    zoomKey = false;
                    panKey = false;
                    mouseDown = false;
                }
            });
        }
        registerMouseHandlers();
        plannedPath.initScale(); 	
}



window.onload = function () {
    rosconnection();
    nav();
}