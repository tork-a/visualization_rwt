$(function() {
  var IMAGE_TOPIC = "/image";
  var HOSTNAME = location.hostname;
  var PORT = "8888";
  var DIV_ID = "canvas-area";
  var WIDTH = 640.0;
  var HEIGHT = 480.0;
  var mjpeg_canvas = new MJPEGCANVAS.Viewer({
    divID : DIV_ID,
    host : HOSTNAME,
    topic : IMAGE_TOPIC,
    width: $("#" + DIV_ID).width(),
    height: $("#" + DIV_ID).width() * HEIGHT / WIDTH
  });
});
