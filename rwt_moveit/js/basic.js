
/**
 * Setup all visualization elements when the page is loaded. 
 */

var fixed_frame;
var current_group;
var link_group;
var end_effector_link;
var start_initial_flag = true;
var goal_initial_flag = true;
var joint_states;
var start_joint_states;
var goal_joint_states;
var start_im_client;
var goal_im_client;
var tfClient;

function init() {
    // Connect to ROS.
    var url = 'ws://' + location.hostname + ':9090';

    var real_ros = new ROSLIB.Ros({
        url : url
    });
    var virtual_ros = new ROSLIB.Ros({
        url : url
    });

    var joint_ros = new ROSLIB.Ros({
        url : url
    });

    joint_names = new ROSLIB.Param({
        ros: joint_ros,
        name: '/joint'
    });

    start_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/update_start_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    goal_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/update_goal_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    im_size_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/im_size/update',
        messageType: 'std_msgs/Float32'
    });

    moveit_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/moveit_joint',
        messageType: 'rwt_moveit/MoveGroupPlan'
    });

    execute_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/execute_trajectory',
        messageType: 'std_msgs/Empty'
    });


    joint_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/update_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });

    computefkClient = new ROSLIB.Service({
        ros : joint_ros,
        name : '/compute_fk',
        serviceType : 'moveit_msgs/GetPositionFK'
    });

    start_initial_interactive_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/start/initial_marker',
        messageType: 'std_msgs/String'
    });

    goal_initial_interactive_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/goal/initial_marker',
        messageType: 'std_msgs/String'
    });

    start_interactive_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/start/marker/feedback',
        messageType: 'visualization_msgs/InteractiveMarkerFeedback'
    });

    goal_interactive_pub = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/goal/marker/feedback',
        messageType: 'visualization_msgs/InteractiveMarkerFeedback'
    });

    plan_listener = new ROSLIB.Topic({
        ros: joint_ros,
        name: '/stock_joint_position',
        messageType: 'std_msgs/Float64MultiArray'
    });


    // Create the main viewer.
    var width = parseInt($("#main-content").css("width"));
    var height = Math.max(
        $(document).height(),
        $(window).height(),
        /* For opera: */
        document.documentElement.clientHeight
    );

    var viewer = new ROS3D.Viewer({
        divID : 'urdf',
        width : width * 0.8,
        height : height * 0.8,
        antialias : true
    });

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());


    fixed_frame_param = new ROSLIB.Param({
        ros: real_ros,
        name: '/fixed_frame'
    });

    link_group_param = new ROSLIB.Param({
        ros: virtual_ros,
        name: '/link_group/'
    });

    end_effector_link_param = new ROSLIB.Param({
        ros: joint_ros,
        name: '/end_effector_link/'
    });

    // Setup listener
    var joint_listener = new ROSLIB.Topic({
        ros : virtual_ros,
        name : '/joint_states',
        messageType : 'sensor_msgs/JointState'
    });

    var goal_listener = new ROSLIB.Topic({
        ros : virtual_ros,
        name : '/goal_joint_states',
        messageType : 'sensor_msgs/JointState'
    });

    var start_listener = new ROSLIB.Topic({
        ros : virtual_ros,
        name : '/start_joint_states',
        messageType : 'sensor_msgs/JointState'
    });


    // Setup a client to listen to TFs.
    fixed_frame_param.get(function(value) {
        fixed_frame = value;
        tfClient = new ROSLIB.TFClient({
            ros : real_ros,
            fixedFrame : fixed_frame,
            angularThres : 0.01,
            transThres : 0.01,
            rate : 10.0
        });
        // Setup the marker client.
        start_im_client = new ROS3D.InteractiveMarkerClient({
            ros : real_ros,
            tfClient : tfClient,
            hidden : true,
            topic : '/start/marker',
            camera : viewer.camera,
            rootObject : viewer.selectableObjects
        });
        goal_im_client = new ROS3D.InteractiveMarkerClient({
            ros : real_ros,
            tfClient : tfClient,
            hidden : true,
            topic : '/goal/marker',
            camera : viewer.camera,
            rootObject : viewer.selectableObjects
        });
    });

    link_group_param.get(function(value) {
        link_group = value;
        for (group_name in link_group) {
            $('#group').append("<option value=" + group_name + ">" + group_name + "</option>");
        }
        $("select#group").bind('change', function() {
            var selector = $("select#group option");
            selector.each(function() {
                $("#" + $(this).val()).hide();
            });
            var group = $("select#group option:selected").val();
            current_group = group;
            $("#" + group).show();
            var msg = new ROSLIB.Message({
                data: current_group
            });
            // inform curren group
            start_initial_interactive_pub.publish(msg);
            goal_initial_interactive_pub.publish(msg);
            start_initial_flag = true;
            goal_initial_flag = true;
            create_joint_position_msg(1, true);
        });

        setTimeout(function() {
            createSliderView();

            joint_listener.subscribe(function(message) {
                joint_states = message;
            });


            start_listener.subscribe(function(message) {
                start_joint_states = message;
                if($('input[name="manip"]:checked').val() != 0) return;

                var fk_link_name;

                if (end_effector_link[current_group] == null) {
                    fk_link_name = "Flange";
                }
                else {
                    fk_link_name = end_effector_link[current_group];
                }

                // Update interactive marker poisition
                var request = new ROSLIB.ServiceRequest({
                    header: {
                        seq: 0,
                        stamp: 0,
                        frame_id: fixed_frame
                    },
                    fk_link_names: [fk_link_name],
                    robot_state: {
                        joint_state: start_joint_states
                    }
                });

                computefkClient.callService(request, function(result) {
                    
                    var interactive_msg = new ROSLIB.Message({
                        marker_name: "start",
                        event_type: 0,
                        pose: {
                            position: {
                                x: result.pose_stamped[0].pose.position.x,
                                y: result.pose_stamped[0].pose.position.y,
                                z: result.pose_stamped[0].pose.position.z
                            },
                            orientation: {
                                x: result.pose_stamped[0].pose.orientation.x,
                                y: result.pose_stamped[0].pose.orientation.y,
                                z: result.pose_stamped[0].pose.orientation.z,
                                w: result.pose_stamped[0].pose.orientation.w
                            }
                        }
                    });
                    start_interactive_pub.publish(interactive_msg);
                });


                for (i = 0; i < start_joint_states.name.length; i++) {
                    var joint_name, joint_num;
                
                    for (j = 0; j < link_group[current_group].length; j++) {
                        if (link_group[current_group][j] == start_joint_states.name[i]) {
                            var min = $('input#' + link_group[current_group][j]).attr("min");
                            var max = $('input#' + link_group[current_group][j]).attr("max");
                            var percent = parseInt((start_joint_states.position[i] - min)/(max - min) * 100);
                            $('input#' + link_group[current_group][j]).attr("value", start_joint_states.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("div").attr("style", "width: " + percent + "%;");
                            $('input#' + link_group[current_group][j]).next().children("a").attr("aria-valuenow", start_joint_states.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("aria-valuetext", 0.5);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("title", start_joint_states.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("style", "left: " +  percent + "%;");
                            $('input#' + link_group[current_group][j]).attr("value", start_joint_states.position[i]);
                            break;
                        }
                    }
                }
                                              

            });

            goal_listener.subscribe(function(message) {
                goal_joint_states = message;
                if($('input[name="manip"]:checked').val() != 1) return;

                var fk_link_name;

                if (end_effector_link[current_group] == null) {
                    fk_link_name = "Flange";
                }
                else {
                    fk_link_name = end_effector_link[current_group];
                }

                // Update interactive marker poisition
                var request = new ROSLIB.ServiceRequest({
                    header: {
                        seq: 0,
                        stamp: 0,
                        frame_id: fixed_frame
                    },
                    fk_link_names: [fk_link_name],
                    robot_state: {
                        joint_state: goal_joint_states
                    }
                });

                computefkClient.callService(request, function(result) {
                        
                    var interactive_msg = new ROSLIB.Message({
                        marker_name: "goal",
                        event_type: 0,
                        pose: {
                            position: {
                                x: result.pose_stamped[0].pose.position.x,
                                y: result.pose_stamped[0].pose.position.y,
                                z: result.pose_stamped[0].pose.position.z
                            },
                            orientation: {
                                x: result.pose_stamped[0].pose.orientation.x,
                                y: result.pose_stamped[0].pose.orientation.y,
                                z: result.pose_stamped[0].pose.orientation.z,
                                w: result.pose_stamped[0].pose.orientation.w
                            }
                        }
                    });
                    goal_interactive_pub.publish(interactive_msg);
                    goal_initial_flag = false;
                });

                for (i = 0; i < goal_joint_states.name.length; i++) {
                    var joint_name, joint_num;
                
                    for (j = 0; j < link_group[current_group].length; j++) {
                        if (link_group[current_group][j] == goal_joint_states.name[i]) {
                            var min = $('input#' + link_group[current_group][j]).attr("min");
                            var max = $('input#' + link_group[current_group][j]).attr("max");
                            var percent = parseInt((goal_joint_states.position[i] - min)/(max - min) * 100);
                            $('input#' + link_group[current_group][j]).attr("value", goal_joint_states.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("div").attr("style", "width: " + percent + "%;");
                            $('input#' + link_group[current_group][j]).next().children("a").attr("aria-valuenow", goal_joint_states.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("aria-valuetext", 0.5);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("title", goal_joint_states.position[i]);
                            $('input#' + link_group[current_group][j]).next().children("a").attr("style", "left: " +  percent + "%;");
                            $('input#' + link_group[current_group][j]).attr("value", goal_joint_states.position[i]);
                            break;
                        }
                    }
                }


                                              

            });

            create_joint_position_msg(1, true);
        }, 3000);

        setTimeout(function() {
            // Setup the URDF client.
            var goalState = new ROS3D.UrdfClient({
                ros : virtual_ros,
                tfPrefix : 'goal',
                color : 0xff3000,
                tfClient : tfClient,
                hidden : true,
                param : 'robot_description',
                rootObject : viewer.scene
            });

            var urdfClient = new ROS3D.UrdfClient({
                ros : real_ros,
                tfClient : tfClient,
                param : 'robot_description',
                rootObject : viewer.scene
            });

            var startState = new ROS3D.UrdfClient({
                ros : joint_ros,
                tfPrefix : 'start',
                color : 0x00df00,
                tfClient : tfClient,
                hidden : true,
                param : 'robot_description',
                rootObject : viewer.scene
            });

            $('#start_state').change(function() {
                if($(this).is(':checked')) {
                    if ($('input[name="manip"]:radio')[0].checked == true) {
                        start_im_client.showIntMarker('start');
                    }
                    startState.add();
                }
                else {
                    start_im_client.hideIntMarker('start');
                    startState.remove();
                }
            });

            $('#goal_state').change(function() {
                if($(this).is(':checked')) {
                    if ($('input[name="manip"]:radio')[1].checked == true) {
                        goal_im_client.showIntMarker('goal');
                    }
                    goalState.add();
                }
                else {
                    goal_im_client.hideIntMarker('goal');
                    goalState.remove();
                }
            });

            $('input[name="manip"]:radio').change(function() {
                if($(this).val() == 0){
                    if($('#start_state').is(':checked')) {
                        start_im_client.showIntMarker('start');
                    }
                    goal_im_client.hideIntMarker('goal');
                }else {
                    if($('#goal_state').is(':checked')) {
                        goal_im_client.showIntMarker('goal');
                    }
                    start_im_client.hideIntMarker('start');
                }
            });

        }, 1500);
    });

    end_effector_link_param.get(function(value) {
        end_effector_link = value;
    });

    plan_listener.subscribe(function(message) {
        message_stock.push(message);
    });

    $("button#init").click(function() {

        positions = new Array();
        dims = new Array();

        $("#" + current_group).children("label").each(function() {
            var dim = new ROSLIB.Message({
                label: ($(this).attr("id").split("-")[0]),
                size: ($(this).attr("id").split("-")[0]).length,
                stride: ($(this).attr("id").split("-")[0]).length
            });
            dims.push(dim);
            for (var i = 0; i < joint_states.name.length;i++) {
                if (joint_states.name[i] == dim.label) {
                    positions.push(joint_states.position[i]);
                    break;
                }
            }
        });

        var msg;
        msg = new ROSLIB.Message({
            layout: {
                dim: dims,
                data_offset: 0
            },
            data: positions
        });
        if($('input[name="manip"]:checked').val() == 0) {
            start_pub.publish(msg);
        }
        else {
            goal_pub.publish(msg);
        }
    });

    $("button#preview").click(function() {
        if(message_stock != null) {
            var i = 0;
            var tmp_start_joint_states = start_joint_states;

            timer = setInterval(function() {
                start_pub.publish(message_stock[i]);
                i++;
                if(i == message_stock.length) {

                    positions = new Array();
                    dims = new Array();

                    $("#" + current_group).children("label").each(function() {
                        var dim = new ROSLIB.Message({
                            label: ($(this).attr("id").split("-")[0]),
                            size: ($(this).attr("id").split("-")[0]).length,
                            stride: ($(this).attr("id").split("-")[0]).length
                        });
                        dims.push(dim);
                        for (var i = 0; i < tmp_start_joint_states.name.length;i++) {
                            if (tmp_start_joint_states.name[i] == dim.label) {
                                positions.push(tmp_start_joint_states.position[i]);
                                break;
                            }
                        }
                    });

                    var msg;
                    msg = new ROSLIB.Message({
                        layout: {
                            dim: dims,
                            data_offset: 0
                        },
                        data: positions
                    });
                    
                    start_pub.publish(msg);
                    clearInterval(timer);
                }
            },100);
        }
    });

    $("button#moveit").click(function() {
        var msg = create_joint_position_msg(0,false);
        moveit_pub.publish(msg);
    });

    $("button#plan").click(function() {
        message_stock = new Array();
        var msg = create_joint_position_msg(0,true);
        moveit_pub.publish(msg);
    });

    $("button#execute").click(function() {
        if(message_stock != null) {
            sim_mode = new ROSLIB.Param({
                ros: joint_ros,
                name: '/sim_mode'
            });
            sim_mode.get(function(value) {
                if (value == true) {
                    timer = setInterval("joint_publish()",100);
                }
                else {
                    var msg = new ROSLIB.Message({
                    });
                    execute_pub.publish(msg);
                }                
            });
        }
    });
}

function create_joint_position_msg(type, plan_only) {

    positions = new Array();
    start_positions = new Array();
    goal_positions = new Array();
    dims = new Array();

    $("#" + current_group).children("label").each(function() {
        var dim = new ROSLIB.Message({
            label: ($(this).attr("id").split("-")[0]),
            size: ($(this).attr("id").split("-")[0]).length,
            stride: ($(this).attr("id").split("-")[0]).length
        });
        dims.push(dim);
        if (type == 0) {
            for (var i = 0; i < start_joint_states.name.length;i++) {
                if (start_joint_states.name[i] == dim.label) {
                    start_positions.push(start_joint_states.position[i]);
                    goal_positions.push(goal_joint_states.position[i]);
                    break;
                }
            }
        }
        else {
        positions.push(parseFloat($(this).next().children("input").next().children("a").attr("aria-valuenow")));
        }
    });

    var msg;
    if (type == 0) {
        msg = new ROSLIB.Message({
            start_joint: {
                layout: {
                    dim: dims,
                    data_offset: 0
                },
                data: start_positions
            },
            goal_joint: {
                layout: {
                    dim: dims,
                    data_offset: 0
                },
                data: goal_positions
            },
            plan_only: plan_only,
            group_name: current_group
        });
    }
    else {
        msg = new ROSLIB.Message({
            layout: {
                dim: dims,
                data_offset: 0
            },
            data: positions
        });
    }
    return msg;
}

function callback() {
    var msg = create_joint_position_msg(1, true);
    if($('input[name="manip"]:checked').val() == 0) {
        start_pub.publish(msg);
    }
    else {
        goal_pub.publish(msg);
    }
}

function joint_publish() {
    if(message_stock.length == 0) {
        clearInterval(timer);
    }
    else {
        joint_pub.publish(message_stock.shift());
    }
}

// create joint_publisher
function createSliderView() {
    var i = 0;
    for (group_name in link_group) {
        $("#slider-pane").append('<div id="' + group_name + '"/>');
        if (i != 0) {
            $("#" + group_name).hide();
        }
        else {
            current_group = group_name;
            i++;
        }
    }
    joint_names.get(function(value) {
        names = value.names;
        for (group_name in link_group) {
            for (var i = 0;i < names.length;i++) {
                if (link_group[group_name].indexOf(names[i]) != -1) {
                    child = $('<label>', {for: names[i], text: names[i]});
                    child2 = $('<input>', {type: "range", name: names[i], id: names[i], value: 0, max: eval("value." + names[i] + ".max"), min: eval("value." + names[i] + ".min"), step: 0.000001, "data-highlight": "true", "data-mini": "true",
                                   onchange: "callback()"});
                    $("#" + group_name).append(child);
                    $("#" + group_name).append(child2);
                }
            }
        }
        $.getScript("js/jquery-mobile/jquery.mobile-1.3.2.min.js");
        var msg = new ROSLIB.Message({
            data: current_group
        });
        start_initial_interactive_pub.publish(msg);
        goal_initial_interactive_pub.publish(msg);
    });
}

function im_size_callback() {
    var size = parseFloat($("#im-size").val());
    var msg = new ROSLIB.Message({
        data: size
    });

    if ($('input[name="manip"]:radio')[0].checked == true && $('#start_state').is(':checked')) {
        im_size_pub.publish(msg);
        start_im_client.hideIntMarker('start');
        setTimeout(function() {
            start_im_client.showIntMarker('start');
        }, 500);
    }
    else if($('input[name="manip"]:radio')[1].checked == true && $('#goal_state').is(':checked')){
        im_size_pub.publish(msg);
        goal_im_client.hideIntMarker('goal');
        setTimeout(function() {
            goal_im_client.showIntMarker('goal');
        }, 500);
    }
}