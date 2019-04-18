var manager = null;
var lin = 0.0;
var ang = 0.0;
var linvel = 0.5;
var angvel = 1.0;
var ros;
var cmd_vel;
var twist;
var cameratopic = "/camera/rgb/image_raw";
var video;

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

function createJoystick() {
    if (manager == null) {
        joystickContainer = document.getElementById('joystick');
        var options = {
            zone: joystickContainer,
            color: "#FF0000",
            size: 150,
            position: { top: 375 + 'px', left: 15 + '%'},
            mode: 'static',
            restJoystick: true

        };

        manager = nipplejs.create(options);

        manager.on('move', function (evt, data) {
        try {
            var dist = data.distance;
            var angle = data.angle.degree;
        }
        catch(error){
            createJoystick();
            console.error(error);
        }

        lin = Math.sin(angle / 57.29) * dist/100.0 * linvel;
        ang = Math.cos(angle / 57.29) * dist/100.0 * -angvel;
        move();
        }); 

        manager.on('end', function () {
            lin = 0.0;
            ang = 0.0;
            move();
        });     
        
    }
}

function initvelocitypublisher() {
    cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

    twist = new ROSLIB.Message({
        linear : {
            x : 0.0,
            y : 0.0,
            z : 0.0
            },
        angular : {
            x : 0.0,
            y : 0.0,
            z : 0.0
            }
    });
   
}
  
function move() {
    twist.linear.x = lin;
    twist.angular.z = ang;
    cmdVel.publish(twist);
}

function buttons() {
    var bl1 = document.getElementById("bl1");
    var bl2 = document.getElementById("bl2");
    var ba1 = document.getElementById("ba1");
    var ba2 = document.getElementById("ba2");

    var lindisp= document.getElementById("lindisp");
    var angdisp= document.getElementById("angdisp");

    bl1.addEventListener("click", function() {
        linvel = linvel*(1.1);
        lindisp.textContent = linvel;
    });
    bl2.addEventListener("click", function() {
        linvel = linvel*(0.9);
        lindisp.textContent = linvel;
    });
    ba1.addEventListener("click", function() {
        angvel = angvel*(1.1);
        angdisp.textContent = angvel;
    });
    ba2.addEventListener("click", function() {
        angvel = angvel*(0.9);
        angdisp.textContent = angvel;
    });
}

function videoon() {
    var topic = document.querySelector("input");
    topic.addEventListener("change", function (){
        cameratopic = topic.value;
        console.log(cameratopic);
    })

    video = document.getElementById("video");
    
    var lv = document.getElementById("lv");
    
    lv.addEventListener("click", function() {
        video.src = "http://localhost:8080/stream?topic=" + cameratopic +"&type=mjpeg&width=600&height=400";
    })
    
    
}

function viewMap() {
    var viewer = new ROS2D.Viewer({
        divID : 'map',
        width : 600,
        height : 500,
	background : '#FFFFFF'
    });

    var gridClient = new ROS2D.OccupancyGridClient({
        ros : ros,
        rootObject : viewer.scene,
        continuous : true
    });
	
    var zoomView = new ROS2D.ZoomView({
	rootObject : viewer.scene
    });
		
    var panView = new ROS2D.PanView({
	rootObject : viewer.scene
    });

    var temp = true;
    gridClient.on('change', function() {
      if(temp == true) {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        temp = false;
    }  
      registerMouseHandlers();
      viewPose();
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

    function viewPose() {
        var poseTopic = new ROSLIB.Topic({
            ros : ros,
            name : '/robot_pose',
            messageType : 'geometry_msgs/Pose'
        });
    
        var robotMarker = new ROS2D.NavigationArrow({
            size : 0.2,
            strokeSize : 0.01,
            pulse : true
    
        });
    
        poseTopic.subscribe(
            function (pose) {
                robotMarker.x = pose.position.x;
                robotMarker.y = -pose.position.y;
                var quaZ = pose.orientation.z;
                var degreeZ = 0;
                if( quaZ >= 0 ) {
                    degreeZ = quaZ / 1 * 180
                } 
                else {
                    degreeZ = (-quaZ) / 1 * 180 + 180
                };
                robotMarker.rotation = -degreeZ + 35;
            }
        )

        gridClient.rootObject.addChild(robotMarker);
    
    }
}



window.onload = function () {
    rosconnection();
    initvelocitypublisher();
    createJoystick();
    buttons();
    videoon();
    viewMap();
}
