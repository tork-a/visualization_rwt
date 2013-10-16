
$(function() {

    // ROSLIB extention
    /**
     * Retrieves a type of ROS topic.
     *
     * @param callback - function with params:
     *   * type - String of the topic type
     */
    ROSLIB.Ros.prototype.getTopicType = function(topic, callback) {
        var topicTypeClient = new ROSLIB.Service({
            ros : this,
            name : '/rosapi/topic_type',
            serviceType : 'rosapi/TopicType'
        });
        var request = new ROSLIB.ServiceRequest({
            topic: topic
        });
        topicTypeClient.callService(request, function(result) {
            callback(result.type);
        });
    };

    /**
     * Retrieves a detail of ROS message.
     *
     * @param callback - function with params:
     *   * details - Array of the message detail
     * @param message - String of a topic type
     */
    ROSLIB.Ros.prototype.getMessageDetails = function(message, callback) {
        var messageDetailClient = new ROSLIB.Service({
            ros : this,
            name : '/rosapi/message_details',
            serviceType : 'rosapi/MessageDetails'
        });
        var request = new ROSLIB.ServiceRequest({
            type: message
        });
        messageDetailClient.callService(request, function(result) {
            var typedefs = result.typedefs;
            callback(result.typedefs);
        });
    };
    
    /**
     * Encode a typedefs into a dictionary like `rosmsg show foo/bar`
     * @param type_defs - array of type_def dictionary
     */
    ROSLIB.Ros.decodeTypeDefs = function(type_defs) {
        var type_def_dict = {};
        var the_type = type_defs[0];
        return ROSLIB.Ros._decodeTypeDefs(type_defs[0], type_defs);
    };

    /**
     * Internal function of ROSLIB.Ros.decodeTypeDefs.
     * It calls itself recursively to resolve type definition
     * using hint_defs.
     * @param the_type - array of type_def dictionary
     * @param hint_defs - array of typedefs
     */
    ROSLIB.Ros._decodeTypeDefs = function(the_type, hint_defs) {
        var type_def_dict = {};
        for (var i = 0; i < the_type.fieldnames.length; i++) {
            var array_len = the_type.fieldarraylen[i];
            var field_name = the_type.fieldnames[i];
            var field_type = the_type.fieldtypes[i];
            if (field_type.indexOf("/") === -1) { // check the field_type includes "/" or not
                if (array_len == -1) {
                    type_def_dict[field_name] = field_type;
                }
                else {
                    type_def_dict[field_name] = [field_type];
                }
            }
            else {
                // lookup the name
                var sub_type = false;
                for (var j = 0; j < hint_defs.length; j++) {
                    if (hint_defs[j].type == field_type) {
                        sub_type = hint_defs[j];
                        break;
                    }
                }
                if (sub_type) {
                    var sub_type_result = ROSLIB.Ros._decodeTypeDefs(sub_type, hint_defs);
                    if (array_len == -1) {
                        type_def_dict[field_name] = sub_type_result;
                    }
                    else {
                        type_def_dict[field_name] = [sub_type_result];
                    }
                }
                else {
                    throw "cannot find " + field_type;
                }
                //ROSLIB.Ros._decodeTypeDefs(field_type, hint_defs)
            }
        }
        return type_def_dict;
    };
    
    
    var plot = new ROSLIB.RWTPlot({
        max_data: 100,          // when using timestamp, it is regarded as seconds
        timestamp:false
    });
    
    plot.initializePlot("#plot-area", {
        interaction: {
            redrawOverlayInterval: 100000 / 60
        },
        yaxis: {
            auto_scale: true
        }
    });

    plot.draw();

    function getValFromAccessor(msg, accessor) {
        if (accessor.length == 0) {
            return msg;
        }
        else {
            if (accessor[0].match(/\[[\d]+\]/)) {
                var array_index = parseInt(accessor[0].match(/\[([\d]+)\]/)[1], 10);
                return getValFromAccessor(msg[accessor[0].split("[")[0]][array_index], accessor.slice(1));
            }
            else {
                return getValFromAccessor(msg[accessor[0]], accessor.slice(1));
            }
        }
    };
    
    // subscribe topic
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":9090"
    });
    
    var sub = null;
    $("#topic-form").submit(function(e) {
        e.preventDefault();
        var topic_name = $("#topic-select").val();
        var accessor = $("#field-accessor").val().split("/");
        
        if (sub) {
            console.log("unsubscribe");
            sub.unsubscribe();
        }
        ros.getTopicType(topic_name, function(topic_type) {
            sub = new ROSLIB.Topic({
                ros: ros,
                name: topic_name,
                messageType: topic_type
            });
            plot.clearData();
            sub.subscribe(function(msg) {
                plot.addData(getValFromAccessor(msg, accessor));
                plot.draw();
            });
        });
        return false;
    });
    
    ros.getTopics(function(topics) {
        $("#topic-select").append(_.map(topics, function(topic) {
            return '<option value="' + topic + '">' + topic + "</option>";
        }).join("\n"));
        $("#topic-select").change();
    });
    $("#topic-select").change(function() {
        var topic_name = $("#topic-select").val();
        ros.getTopicType(topic_name, function(topic_type) {
            ros.getMessageDetails(topic_type, function(details) {
                var decoded = ROSLIB.Ros.decodeTypeDefs(details);
                $("#message-detail").find("pre").html(JSON.stringify(decoded, null, "  ")); // pretty print
            });
        });
    });
});
