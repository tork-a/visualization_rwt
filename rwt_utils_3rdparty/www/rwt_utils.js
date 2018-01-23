ROSLIB.Ros.prototype.install_config_button = function(id, auto_connect) {
  var auto_connect = typeof auto_connect !== 'undefined' ? auto_connect : true;
  var that = this;
  var parent = document.getElementById(id);

  var dropdown = document.createElement("span");
  parent.appendChild(dropdown);
  dropdown.setAttribute("class", "dropdown");
  dropdown.style.display = "inline";
  
  var btn = document.createElement("button");
  dropdown.appendChild(btn);

  btn.setAttribute("class", "btn btn-primary dropdown-toggle");
  btn.setAttribute("data-toggle", "dropdown");
  btn.setAttribute("aria-haspopup", true);
  btn.setAttribute("aria-expanded", false);
  btn.id = "button-ros-master-settings";
  btn.type = "button";
  btn.textContent = "Settings ";

  var span = document.createElement("span");
  btn.appendChild(span);
  span.setAttribute("class", "glyphicon glyphicon-wrench");
  span.setAttribute("aria-hidden", true);

  var span = document.createElement("span");
  btn.appendChild(span);
  span.setAttribute("class", "caret");

  var group = document.createElement("ul");
  dropdown.appendChild(group);
  group.setAttribute("class", "dropdown-menu");
  group.setAttribute("aria-labelledby", "button-ros-master-settings");
  group.id = "ul-ros-connect-settings";

  var li = document.createElement("li");
  group.appendChild(li);
  li.onclick = function(e) {
    console.log(e);
    e.stopPropagation();
  };

  var label = document.createElement("label");
  li.appendChild(label);
  label.setAttribute("for", "input-ros-master-uri");
  label.textContent = "ROS Master URI";
  
  var input = document.createElement("input");
  li.appendChild(input);
  input.id = "input-ros-master-uri";
  input.type = "text";
  input.pattern = "^wss?://[a-zA-Z0-9\-\._]+:[0-9]+/$";
  input.required = true;
  input.value = "ws://" + location.hostname + ":8888/";
  input.setAttribute("class", "form-control");
  input.setAttribute("placeholder", "ROS WS Master URI");
  // input.setAttribute("aria-label", "ROS WS Master URI");
  input.oninput = function() {
    var b = document.getElementById("button-ros-master-connect");
    if (input.checkValidity()) {
      b.removeAttribute("disabled");
    } else {
      b.setAttribute("disabled", true);
    }
  };
  
  var span = document.createElement("span");
  li.appendChild(span);
  span.setAttribute("class", "input-group-btn");
  btn = document.createElement("button");
  span.appendChild(btn);
  btn.setAttribute("class", "btn btn-primary");
  btn.id = "button-ros-master-connect";
  btn.type = "button";
  btn.textContent = "Connect";
  btn.onclick = function() {
    var e = document.getElementById("input-ros-master-uri");
    if (that.isConnected) {
      that.close();
    }
    that.connect(e.value);
  };

  // connect with default location
  if (auto_connect) {
    that.connect(input.value);
  }
};

ROSLIB.Ros.prototype.url = function() {
  if (this.socket) {
    return new URL(this.socket.url);
  } else {
    return null;
  }
};
