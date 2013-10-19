ROSLIB.profile = function(func) {
  var before = ROSLIB.Time.now();
  func.call();
  var after = ROSLIB.Time.now();
  var diff = after.substract(before);
  console.log('profile: ' + diff.toSec());
};
