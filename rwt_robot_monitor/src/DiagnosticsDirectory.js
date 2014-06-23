// DiagnosticsDirectory.js

/**
 * @fileOverview a file to define RWTRobotMonitor.DiagnosticsDirectory class
 * @author Ryohei Ueda
 */

/**
 * @class DiagnosticsDirectory
 * @param spec
 */
ROSLIB.DiagnosticsDirectory = function(spec, parent, children) {
  if (typeof parent === 'undefined') {
    parent = null;
  }
  if (typeof children === 'undefined') {
    children = [];
  }

  this.parent = parent;
  this.children = children;
  this.status = spec.status;
  this.name = spec.name;
};

/**
 * lookup the directory whose name equals to `name'. This method digs the children
 * of the directory. If succeeeds to find that, returns the instance
 * of ROSLIB.DiagnosticsDirectory. if failed, returns null.
 * @param name - the name of directory
 */
ROSLIB.DiagnosticsDirectory.prototype.findDirectory = function(name) {
  if (this.name.toString() === name.toString()) {
    return this;
  }
  else if (this.children.length === 0) { // no children
    return null;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].findDirectory(name);
      if (child_result !== null) {
        return child_result;
      }
    }
    return null;
  }
};

/**
 * add child directory to this directory
 * @param directory - an instance of ROSLIB.DiagnosticsDirectory
 */
ROSLIB.DiagnosticsDirectory.prototype.addChild = function(directory) {
  this.children.push(directory);
  directory.parent = this;
};

/**
 * create a child directory which has this directory as parent
 * @ param name - name of child directory
 */
ROSLIB.DiagnosticsDirectory.prototype.createChild = function(name) {
  var child = new ROSLIB.DiagnosticsDirectory({
    name: name
  }, this);
  this.addChild(child);
  return child;
};

/**
 * register a status to the directory
 * @param status - instance of ROSLIB.DiagnosticsStatus
 */
ROSLIB.DiagnosticsDirectory.prototype.registerStatus = function(status) {
  this.status = status;
  return status;
};

/**
 * return the instance of directory if the directory has error instance 
 * as children.
 */
ROSLIB.DiagnosticsDirectory.prototype.isChildrenHasError = function() {
  if (this.isErrorStatus()) {
    return this;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].isChildrenHasError();
      if (child_result) {
        return child_result;
      }
    }
  }
};

/**
 * return true if the status registered to the directory has error level.
 */
ROSLIB.DiagnosticsDirectory.prototype.isErrorStatus = function() {
  if (this.status) {
    return this.status.isERROR();
  }
  else {
    return false;
  }
};

/**
 * return full path of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.fullName = function() {
  var rec = function(target_dir) {
    if (target_dir.parent === null) { // root
      return '';
    }
    else {
      var parent_result = rec(target_dir.parent);
      return parent_result + '/' + target_dir.name;
    }
  };
  return rec(this);
};

/**
 * get the uniq id which is available as html id
 */
ROSLIB.DiagnosticsDirectory.prototype.uniqID = function() {
  var rec = function(target_dir) {
    if (target_dir.parent === null) { // root
      return '';
    }
    else {
      var parent_result = rec(target_dir.parent);
      if (parent_result.length === 0) {
        return target_dir.name;
      }
      else {
        return parent_result + '-' + target_dir.name;
      }
    }
  };
  return rec(this).replace(' ', '-').replace('(', '').replace(')', '').replace('\t', '-');
};


/**
 * get an array of all the directories without root itself
 */
ROSLIB.DiagnosticsDirectory.prototype.getAllDirectoriesWithoutRoot = function() {
  var self = this;
  var directories = self.getAllDirectories();
  _.remove(directories, function(dir) {
    return dir === self;
  });
  return directories;
};

/**
 * get an array of all the directories
 */
ROSLIB.DiagnosticsDirectory.prototype.getAllDirectories = function() {
  var rec = function(target_dir) {
    if (target_dir.children.length === 0) {
      return [target_dir];
    }
    else {
      var result = [];
      for (var i = 0; i < target_dir.children.length; i++) {
        var child_result = rec(target_dir.children[i]);
        result = result.concat(child_result);
      }
      result.push(target_dir);
      return result;
    }
  };
  return rec(this);
};


/**
 * get an array of directories which has `level' such as error, warning and ok.
 */
ROSLIB.DiagnosticsDirectory.prototype.getDirectories = function(level) {
  var rec = function(target_dir) {
    if (target_dir.children.length === 0) {
      if (target_dir.status && target_dir.status.level === level) {
        return [target_dir];
      }
      else {
        return [];
      }
    }
    else {
      var result = [];
      for (var i = 0; i < target_dir.children.length; i++) {
        var child_result = rec(target_dir.children[i]);
        result = result.concat(child_result);
      }
      if (target_dir.status && target_dir.status.level === level) {
        result.push(target_dir);
      }
      return result;
    }
  };
  return rec(this);
};

/**
 * return an array of directories which has error status
 */
ROSLIB.DiagnosticsDirectory.prototype.getErrorDirectories = function() {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.ERROR);
};

/**
 * return an array of directories which has warn status
 */
ROSLIB.DiagnosticsDirectory.prototype.getWarnDirectories = function() {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.WARN);
};

/**
 * return an array of directories which has ok status
 */
ROSLIB.DiagnosticsDirectory.prototype.getOkDirectories = function() {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.OK);
};

/**
 * look up the directory by name throught directory tree return the instance of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.findByName = function(name) {
  if (this.status && this.status.name.toString() === name.toString()) {
    return this;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].findByName(name);
      if (child_result) {
        return child_result;
      }
    }
    return null;
  }
};

/**
 * return html to show icon suitable for error status of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.getIconHTML = function() {
  if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.OK) {
    return '<span class="glyphicon-ok glyphicon"></span>';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.WARN) {
    return '<span class="glyphicon-exclamation-sign glyphicon"></span>';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.ERROR) {
    return '<span class="glyphicon-minus-sign glyphicon"></span>';
  }
};

/**
 * return html of icon to show this directory has child
 */
ROSLIB.DiagnosticsDirectory.prototype.getCollapseIconHTML = function() {
  if (this.children.length !== 0) {
    return '<span class="glyphicon-chevron-right glyphicon"></span>';
  }
  else {
    return '';
  }
};


/**
 * return true if the directory has any children
 */
ROSLIB.DiagnosticsDirectory.prototype.hasChildren = function() {
  return this.children.length !== 0;
};
