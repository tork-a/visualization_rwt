var manager = null;
var lin = 0;
var ang = 0;
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
            size: 200,
            
            mode: 'static',
            restJoystick: true

        };

        manager = nipplejs.create(options);

        manager.on('move', function (evt, data) {
        try {
            var dirx = data.direction.x;
            var diry = data.direction.y;
            var dist = data.distance;
        }
        catch(error){
            createJoystick();
            console.error(error);
        }

        if(diry == "up") {
            lin=linvel * dist/100;
        }
        if(diry == "down") {
            lin = -linvel * dist/100 ;
        }
        if(dirx == "right"){
            ang = -angvel * dist/100;
        }
        if(dirx == "left"){
            ang = angvel * dist/100;
        }
        move();
        });

        manager.on('end', function () {
            lin=0;
            ang=0;
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
            x : 0,
            y : 0,
            z : 0
            },
        angular : {
            x : 0,
            y : 0,
            z : 0
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
        video.src = "http://localhost:8080/stream?topic=" + cameratopic +"&type=mjpeg&quality=80";
    })
    
    
}
window.onload = function () {
    rosconnection();
    initvelocitypublisher();
    createJoystick();
    buttons();
    videoon();
}
